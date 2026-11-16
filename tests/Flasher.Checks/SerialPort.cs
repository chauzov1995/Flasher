// The checks compile the WPF sources against this in-memory port. No hardware is opened.
using System.Collections.Generic;
using System.IO;

namespace System.IO.Ports;

public sealed class SerialDataReceivedEventArgs : EventArgs { }
public delegate void SerialDataReceivedEventHandler(object sender, SerialDataReceivedEventArgs args);

public sealed class SerialPort(string name, int baud) : IDisposable
{
    internal static string[] Ports = { "COM91", "COM92" };
    internal static readonly Dictionary<string, SerialPort> Active = new();
    internal static string BusyPort;
    private string buffer = "";

    public static string[] GetPortNames() => Ports;
    public event SerialDataReceivedEventHandler DataReceived;
    public void Open()
    {
        if (name == BusyPort) throw new IOException("Порт занят");
        if (baud != 115200) throw new ArgumentOutOfRangeException(nameof(baud));
        Active.Add(name, this);
    }
    internal void Feed(string data)
    {
        buffer += data;
        DataReceived?.Invoke(this, new SerialDataReceivedEventArgs());
    }
    public string ReadExisting()
    {
        var data = buffer;
        buffer = "";
        return data;
    }
    public void Dispose()
    {
        if (Active.TryGetValue(name, out var active) && active == this) Active.Remove(name);
    }
}
