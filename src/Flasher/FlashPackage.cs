using System;
using System.Buffers.Binary;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Security.Cryptography;

namespace WpfApp1;

internal sealed class FlashPackage : IDisposable
{
    private const int FlashSize = 4 * 1024 * 1024;
    // boot_app0.bin is shared by all Arduino kits in this distribution.
    private const string InitialOtaHash = "F94C5D786A7A8FAB06AC5D10E33BF37711A6697636DC037559EA19CC410A17F0";
    private readonly List<FileStream> files = new();
    public ProcessStartInfo Command { get; }

    public FlashPackage(ProcessStartInfo command, string loader, string table, string ota, string app)
    {
        Command = command;
        try
        {
            ValidateFile(loader, 0x7000, ValidateImage);
            int appCapacity = 0;
            ValidateFile(table, 0x1000, data => appCapacity = ReadPartitions(data));
            ValidateFile(ota, 0x2000, data =>
                Require(Convert.ToHexString(SHA256.HashData(data)) == InitialOtaHash, "повреждены начальные данные OTA"));
            ValidateFile(app, appCapacity, ValidateImage);

            foreach (var value in new[] { "0x1000", loader, "0x8000", table, "0xe000", ota, "0x10000", app })
                command.ArgumentList.Add(value);
        }
        catch
        {
            Dispose();
            throw;
        }
    }

    private void ValidateFile(string path, int maxLength, Action<byte[]> validate)
    {
        var file = File.Open(path, FileMode.Open, FileAccess.Read, FileShare.Read);
        files.Add(file); // Hold the verified files until esptool exits, including while a batch is queued.
        try
        {
            Require(file.Length > 0 && file.Length <= maxLength, "размер файла не соответствует области записи");
            var bytes = new byte[(int)file.Length];
            file.ReadExactly(bytes);
            validate(bytes);
        }
        catch (InvalidDataException ex)
        {
            throw new InvalidDataException($"{Path.GetFileName(path)} — {ex.Message}", ex);
        }
    }

    private static void ValidateImage(byte[] data)
    {
        Require(data.Length >= 32 && data[0] == 0xe9 && data[1] is >= 1 and <= 16 &&
                U16(data, 12) == 0 && data[23] <= 1, "некорректный заголовок образа ESP32");
        int pos = 24;
        byte checksum = 0xef;
        for (int i = 0; i < data[1]; i++)
        {
            Require(pos + 8 <= data.Length, "неполный заголовок сегмента");
            uint length = U32(data, pos + 4);
            pos += 8;
            Require(length <= data.Length - pos && length % 4 == 0, "некорректная длина сегмента");
            foreach (var value in data.AsSpan(pos, (int)length)) checksum ^= value;
            pos += (int)length;
        }

        int bodyLength = (pos / 16 + 1) * 16;
        bool hasHash = data[23] == 1;
        Require(data.Length == bodyLength + (hasHash ? 32 : 0), "неверная длина образа");
        Require(checksum == data[bodyLength - 1], "не совпала контрольная сумма сегментов");
        if (hasHash)
            Require(data.AsSpan(bodyLength).SequenceEqual(SHA256.HashData(data.AsSpan(0, bodyLength))), "не совпал SHA-256 образа");
    }

    private static int ReadPartitions(byte[] data)
    {
        Require(data.Length % 32 == 0, "неполная запись таблицы разделов");
        var ranges = new List<(uint Start, uint End)>();
        int appCapacity = 0;
        bool otaFound = false;
        for (int pos = 0; pos < data.Length; pos += 32)
        {
            int magic = U16(data, pos);
            if (magic == 0xffff || magic == 0xebeb)
            {
                if (magic == 0xebeb)
                {
                    Require(data.AsSpan(pos + 2, 14).IndexOfAnyExcept((byte)0xff) < 0 &&
                        data.AsSpan(pos + 16, 16).SequenceEqual(MD5.HashData(data.AsSpan(0, pos))), "не совпал MD5 таблицы разделов");
                    pos += 32;
                }
                Require(data.AsSpan(pos).IndexOfAnyExcept((byte)0xff) < 0, "лишние данные после таблицы разделов");
                break;
            }
            Require(magic == 0x50aa, "неверная сигнатура раздела");
            byte type = data[pos + 2], subtype = data[pos + 3];
            uint start = U32(data, pos + 4), size = U32(data, pos + 8);
            Require(start >= 0x9000 && start < FlashSize && size > 0 && size <= FlashSize - start &&
                    start % (type == 0 ? 0x10000 : 0x1000) == 0, "некорректные границы раздела");
            Require(!ranges.Any(range => start < range.End && range.Start < start + size), "разделы пересекаются");
            ranges.Add((start, start + size));
            if (type == 0 && start == 0x10000 && subtype is 0 or 0x10) appCapacity = (int)size;
            if (type == 1 && subtype == 0 && start == 0xe000 && size >= 0x2000) otaFound = true;
        }
        Require(appCapacity > 0 && otaFound, "таблица не соответствует адресам приложения и OTA");
        return appCapacity;
    }

    private static ushort U16(byte[] bytes, int offset) => BinaryPrimitives.ReadUInt16LittleEndian(bytes.AsSpan(offset));
    private static uint U32(byte[] bytes, int offset) => BinaryPrimitives.ReadUInt32LittleEndian(bytes.AsSpan(offset));
    private static void Require(bool condition, string message)
    {
        if (!condition) throw new InvalidDataException(message);
    }

    public void Dispose()
    {
        foreach (var file in files) file.Dispose();
    }
}
