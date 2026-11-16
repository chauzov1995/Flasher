using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text.Json.Serialization;

namespace WpfApp1;

public enum FlashStage { Idle, Writing, Success, Failure }

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
    [JsonIgnore] public bool Ready => SelectedProfile != null && portNames?.Contains(Port) == true;
    [JsonIgnore] public FlashStage Stage { get; private set; }
    [JsonIgnore] public string flMAC
    {
        get => mac;
        set { mac = value; Changed(); }
    }

    [JsonIgnore] public string DisplayStatus => Stage switch
    {
        FlashStage.Writing => "Идёт запись…",
        FlashStage.Success => "Прошито",
        FlashStage.Failure => error,
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
        Changed(nameof(Stage));
        Changed(nameof(DisplayStatus));
    }

    private void ChoiceChanged([CallerMemberName] string name = null)
    {
        flMAC = null;
        SetStage(FlashStage.Idle);
        Changed(name);
        Changed(nameof(SelectedProfile));
        Changed(nameof(Ready));
    }

    private void Changed([CallerMemberName] string name = null) =>
        PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(name));
}
