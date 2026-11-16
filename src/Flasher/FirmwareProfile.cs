using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Text.Json;
using System.Text.Json.Serialization;

namespace WpfApp1;

public sealed class FirmwareProfile
{
    public string NameDir { get; set; }
    public string Desc { get; set; }
    public string Kind { get; set; }
    public string BinName { get; set; }
    public string Bootloader { get; set; }
    public string Partitions { get; set; }
    public string Layout { get; set; }
    public bool Erase { get; set; }

    [JsonIgnore]
    public string Label => (Kind switch
    {
        "test" => $"Тест · {Desc ?? NameDir}",
        "factory" => $"Заводская · {Desc ?? NameDir}",
        _ => Desc ?? NameDir
    }) + (Erase ? " (полное стирание)" : "");

    private string directory;
    private static readonly JsonSerializerOptions Options = new()
    {
        PropertyNameCaseInsensitive = true,
        AllowTrailingCommas = true
    };

    public static FirmwareProfile Load(string directory)
    {
        var manifest = Path.Combine(directory, "ino.json");
        var profile = JsonSerializer.Deserialize<FirmwareProfile>(File.ReadAllText(manifest), Options);
        if (profile == null || !string.Equals(profile.NameDir, Path.GetFileName(directory), StringComparison.OrdinalIgnoreCase))
            throw new InvalidDataException($"Поле nameDir должно совпадать с именем папки: {manifest}");
        profile.directory = directory;
        return profile;
    }

    private static string RequiredFile(string folder, string name)
    {
        if (string.IsNullOrWhiteSpace(name) || Path.GetFileName(name) != name ||
            name.IndexOfAny(Path.GetInvalidFileNameChars()) >= 0)
            throw new InvalidDataException($"Некорректное имя файла «{name}» в комплекте {folder}");
        var path = Path.Combine(folder, name);
        if (!File.Exists(path) || new FileInfo(path).Length == 0)
            throw new FileNotFoundException($"Отсутствует или пуст файл: {path}", path);
        return path;
    }

    public ProcessStartInfo CreateFlashCommand(string root, string port, bool erase)
    {
        var tools = Path.Combine(root, "partsheme");
        var layoutDir = Layout switch
        {
            null or "" => directory,
            "1.0.4.rc1" => Path.Combine(root, "partsheme 1.0.4.rc1"),
            _ => throw new InvalidDataException($"Неизвестная схема: {Layout}")
        };

        var command = new ProcessStartInfo(RequiredFile(tools, "esptool.exe"))
        {
            UseShellExecute = false,
            CreateNoWindow = true,
            RedirectStandardOutput = true,
            RedirectStandardError = true
        };
        var args = new List<string>
        {
            "--chip", "esp32", "--port", port, "--baud", "921600", "--before", "default_reset",
            "--after", "hard_reset", "write_flash", "-z", "--flash_mode", "dio", "--flash_freq", "80m",
            "--flash_size", "4MB"
        };
        if (Erase || erase) args.Add("--erase-all");
        args.AddRange(new[]
        {
            "0x1000", RequiredFile(layoutDir, Bootloader),
            "0x8000", RequiredFile(layoutDir, Partitions),
            "0xe000", RequiredFile(tools, "boot_app0.bin"),
            "0x10000", RequiredFile(directory, BinName)
        });
        foreach (var arg in args) command.ArgumentList.Add(arg);
        return command;
    }
}
