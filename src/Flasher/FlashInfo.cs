using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text.Json.Serialization;

namespace WpfApp1;

public enum FlashStage { Idle, Writing, Success, Failure, Historical }

public sealed class FlashInfo : INotifyPropertyChanged
{
    private string path;
    private string port;
    private bool selected;
    private string mac;
    private string error;

    // These names are part of the per-user JSON saved by earlier releases.
    public string Path
    {
        get => path;
        set
        {
            if (path == value) return;
            path = value;
            ChoiceChanged();
        }
    }

    public string Port
    {
        get => port;
        set
        {
            if (port == value) return;
            port = value;
            ChoiceChanged();
        }
    }

    public bool chk
    {
        get => selected;
        set
        {
            if (selected == value) return;
            selected = value;
            Changed();
        }
    }

    [JsonIgnore] public int elem { get; set; }
    [JsonIgnore] public string[] portNames { get; set; }
    [JsonIgnore] public List<FirmwareProfile> DirectoryProject { get; set; }
    [JsonIgnore] public FirmwareProfile SelectedProfile => DirectoryProject?.FirstOrDefault(p => p.NameDir == Path);
    [JsonIgnore] public bool Connected => portNames?.Contains(Port, StringComparer.OrdinalIgnoreCase) == true;
    [JsonIgnore] public string[] PortChoices => Port == null || Connected ? portNames : portNames.Append(Port).ToArray();
    [JsonIgnore] public bool Ready => SelectedProfile != null && SelectedProfile.ValidationError == null && Connected;
    [JsonIgnore] public bool CanEditPreset => Stage == FlashStage.Success && Ready && flMAC != null && LocalPreset.Supports(Path);
    public string LastLogName { get; set; }
    [JsonIgnore] public string LastLog { get; private set; }
    [JsonIgnore] public bool HasLog => LastLog != null;
    public void SetLog(string file)
    {
        LastLog = file;
        LastLogName = System.IO.Path.GetFileName(file);
        Changed(nameof(HasLog));
    }
    public void RestoreLog(string directory)
    {
        if (string.IsNullOrEmpty(LastLogName) || LastLogName != System.IO.Path.GetFileName(LastLogName) ||
            LastLogName.IndexOfAny(System.IO.Path.GetInvalidFileNameChars()) >= 0 ||
            !LastLogName.EndsWith(".txt", StringComparison.OrdinalIgnoreCase)) return;
        var file = System.IO.Path.Combine(directory, LastLogName);
        if (System.IO.File.Exists(file)) SetLog(file);
    }

    [JsonIgnore] public FlashStage Stage { get; private set; }
    [JsonIgnore] public string flMAC
    {
        get => mac;
        set { mac = value; Changed(); Changed(nameof(CanEditPreset)); }
    }

    [JsonIgnore] public string DisplayStatus => Stage switch
    {
        FlashStage.Writing => "Идёт запись…",
        FlashStage.Success => "Прошито",
        FlashStage.Failure or FlashStage.Historical => error,
        _ when SelectedProfile?.ValidationError != null => "Комплект недоступен",
        _ when Port != null && !Connected => "Порт отключён",
        _ when Ready => "Готова к записи",
        _ when !string.IsNullOrEmpty(Port) => "Выберите профиль",
        _ when SelectedProfile != null => "Выберите порт",
        _ => "Не настроена"
    };

    public event PropertyChangedEventHandler PropertyChanged;

    public void SetStage(FlashStage stage, string detail = null)
    {
        Stage = stage;
        error = detail;
        if (stage != FlashStage.Success && stage != FlashStage.Writing) flMAC = null;
        Changed(nameof(CanEditPreset));
        Changed(nameof(Stage));
        Changed(nameof(DisplayStatus));
    }

    public void CatalogChanged()
    {
        Changed(nameof(Connected));
        Changed(nameof(Ready));
        Changed(nameof(DisplayStatus));
        Changed(nameof(CanEditPreset));
    }

    private void ChoiceChanged([CallerMemberName] string name = null)
    {
        flMAC = null;
        SetStage(FlashStage.Idle);
        Changed(name);
        Changed(nameof(SelectedProfile));
        CatalogChanged();
    }

    private void Changed([CallerMemberName] string name = null) =>
        PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(name));
}
