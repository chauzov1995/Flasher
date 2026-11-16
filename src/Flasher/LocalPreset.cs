using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.IO;
using System.Linq;
using System.Text.Json;
using System.Text.Json.Serialization;

namespace WpfApp1;

public sealed class DeviceIOT : INotifyPropertyChanged
{
    private string type;
    public string Type
    {
        get => type;
        set
        {
            if (type == value) return;
            type = value;
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(Type)));
            Describe();
        }
    }

    [JsonIgnore] public string[] TypesDevicesSettings { get; } = { "svet", "retrotop_up", "none" };
    [JsonIgnore] public int num { get; set; }
    public int? numPIN { get; set; }
    public int? numButton { get; set; }
    public int? numPIN2 { get; set; }
    public int? numButton2 { get; set; }
    public bool reverseBULB { get; set; }
    public bool dimBRIGHTNESS { get; set; }
    public bool SMOTH { get; set; }
    public bool IKsenor { get; set; }
    public bool MechanicBtn { get; set; }

    [JsonIgnore] public string Opisanie
    {
        get
        {
            if (Type == "none") return "";
            var parts = new List<string> { $"Выход {numPIN?.ToString() ?? "—"} · Кнопка {numButton?.ToString() ?? "—"}" };
            if (Type == "svet")
            {
                if (reverseBULB) parts.Add("Инверсия выхода");
                if (dimBRIGHTNESS) parts.Add("Регулировка яркости");
                if (SMOTH) parts.Add("Плавное включение");
                if (IKsenor) parts.Add("ИК-датчик");
            }
            if (Type == "retrotop_up")
            {
                parts.Add($"Выход 2: {numPIN2?.ToString() ?? "—"} · Кнопка 2: {numButton2?.ToString() ?? "—"}");
                if (MechanicBtn) parts.Add("Механическая кнопка");
            }
            return string.Join(" · ", parts);
        }
    }

    public event PropertyChangedEventHandler PropertyChanged;
    public void Describe() => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(Opisanie)));

    public static bool ValidGpio(int? pin, bool output) =>
        pin is (>= 0 and <= 5) or (>= 12 and <= 19) or (>= 21 and <= 23) or (>= 25 and <= 27) or 32 or 33 ||
        (!output && pin is >= 34 and <= 39);

    internal bool Valid => Type switch
    {
        "none" => true,
        "svet" => ValidGpio(numPIN, true) && ValidGpio(numButton, false),
        "retrotop_up" => ValidGpio(numPIN, true) && ValidGpio(numButton, false) &&
            ValidGpio(numPIN2, true) && ValidGpio(numButton2, false),
        _ => false
    };
}

internal sealed class LocalPreset
{
    internal static string DataDirectory => Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData), "Flasher");
    internal static bool Supports(string profile) => profile is "SmartKitchen" or "SmartKitchen2";
    internal static bool ValidMac(string value) => value?.Length == 12 && value.All(Uri.IsHexDigit);
    private readonly string path;
    internal List<DeviceIOT> Devices { get; }

    internal LocalPreset(string profile, string mac, string directory)
    {
        if (!Supports(profile) || !ValidMac(mac)) throw new ArgumentException("Нужны профиль SmartKitchen и MAC платы.");
        path = Path.Combine(directory, profile + "-" + mac.ToUpperInvariant() + ".json");
        Devices = File.Exists(path) ? JsonSerializer.Deserialize<List<DeviceIOT>>(File.ReadAllText(path)) : Defaults(profile);
        if (Devices?.Count != 6 || Devices.Any(d => d == null || !d.TypesDevicesSettings.Contains(d.Type)))
            throw new InvalidDataException("Некорректный файл пресета: " + path);
        for (int i = 0; i < Devices.Count; i++) Devices[i].num = i + 1;
    }

    internal void Save()
    {
        foreach (var device in Devices)
            if (!device.Valid) throw new InvalidDataException($"Устройство {device.num}: укажите допустимые GPIO для всех выходов и кнопок.");
        Directory.CreateDirectory(Path.GetDirectoryName(path));
        string temp = path + "." + Guid.NewGuid().ToString("N") + ".tmp";
        try
        {
            File.WriteAllText(temp, JsonSerializer.Serialize(Devices, new JsonSerializerOptions { WriteIndented = true }));
            File.Move(temp, path, true);
        }
        finally { File.Delete(temp); }
    }

    private static List<DeviceIOT> Defaults(string profile)
    {
        static DeviceIOT Light(int? pin, int button) => new()
        {
            Type = "svet", numPIN = pin, numButton = button, reverseBULB = true, dimBRIGHTNESS = true, SMOTH = true
        };
        return new()
        {
            Light(profile == "SmartKitchen" ? 32 : null, 21), Light(33, 22), Light(25, 19), Light(12, 23),
            new() { Type = "retrotop_up", numPIN = 27, numButton = 18, numPIN2 = 26, numButton2 = 5 },
            new() { Type = "none" }
        };
    }
}
