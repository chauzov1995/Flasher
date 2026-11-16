using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Text.Json;
using System.Windows.Controls.Primitives;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;

namespace WpfApp1;

public partial class MainWindow : Window
{
    private const int BoardCount = 5;
    private readonly List<FlashInfo> rows;
    private readonly Properties.Settings settings = Properties.Settings.Default;
    private SerialPort monitor;
    private string serialTail = "";
    private bool flashing;

    public MainWindow()
    {
        InitializeComponent();
        try
        {
            rows = JsonSerializer.Deserialize<List<FlashInfo>>(settings.spifflash);
        }
        catch (JsonException) { }
        rows ??= new List<FlashInfo>();
        rows.RemoveAll(row => row == null);
        while (rows.Count < BoardCount) rows.Add(new FlashInfo());
        chk_erase.IsChecked = settings.erase;
        chk_erase_Copy.IsChecked = settings.monitoe;
        chk_noClose.IsChecked = settings.noClosing;
        RefreshCatalog();
    }

    private void RefreshCatalog()
    {
        var profiles = new List<FirmwareProfile>();
        var messages = new List<string>();
        foreach (var folder in new[] { "projects", "projectsard" })
        {
            var path = Path.Combine(AppContext.BaseDirectory, folder);
            if (!Directory.Exists(path))
            {
                if (folder == "projects") messages.Add($"Отсутствует каталог: {path}");
                continue;
            }
            foreach (var dir in Directory.GetDirectories(path).OrderBy(p => p))
            {
                if (folder == "projectsard" && !File.Exists(Path.Combine(dir, "ino.json"))) continue;
                try
                {
                    var profile = FirmwareProfile.Load(dir);
                    if (profiles.Any(p => p.NameDir.Equals(profile.NameDir, StringComparison.OrdinalIgnoreCase)))
                        throw new InvalidDataException($"Повторяющееся имя: {profile.NameDir}");
                    profiles.Add(profile);
                }
                catch (Exception e) { messages.Add($"{Path.GetFileName(dir)}: {e.Message}"); }
            }
        }

        var ports = SerialPort.GetPortNames().OrderBy(p => p.Length).ThenBy(p => p).ToArray();
        if (ports.Length == 0) messages.Add("COM-порты не обнаружены. Подключите плату и нажмите «Обновить».");
        portSummary.Text = $"COM-порты: {ports.Length}";
        portIndicator.Fill = ports.Length > 0 ? (Brush)FindResource("Accent") : Brushes.DarkGray;
        profileSummary.Text = $"Профили: {profiles.Count}";
        logSource.Text = "Вывод esptool и COM-порта";
        txt_log.Text = string.Join(Environment.NewLine, messages);
        lb.ItemsSource = null;
        for (var i = 0; i < rows.Count; i++)
        {
            var row = rows[i];
            row.PropertyChanged -= Row_Changed;
            row.elem = i + 1;
            row.DirectoryProject = profiles;
            row.portNames = ports;
            if (!ports.Contains(row.Port)) row.Port = null;
            if (!profiles.Any(p => p.NameDir == row.Path)) row.Path = null;
            row.PropertyChanged += Row_Changed;
        }
        lb.ItemsSource = rows;
        UpdateSelection();
    }

    private void Row_Changed(object sender, PropertyChangedEventArgs e)
    {
        if (e.PropertyName == nameof(FlashInfo.chk) || e.PropertyName == nameof(FlashInfo.Ready))
            UpdateSelection();
    }

    private void UpdateSelection()
    {
        var selected = rows.Where(r => r.chk).ToArray();
        chk_all.IsChecked = selected.Length == 0 ? false : selected.Length == rows.Count ? true : null;
        selectionSummary.Text = $"Выбрано: {selected.Length} из {rows.Count}";
        var incomplete = selected.FirstOrDefault(r => !r.Ready);
        var duplicate = selected.Where(r => r.Ready).GroupBy(r => r.Port).FirstOrDefault(g => g.Count() > 1);
        string hint = "";
        if (incomplete != null) hint = $"Строка {incomplete.elem:00}: выберите прошивку и COM-порт.";
        else if (duplicate != null) hint = $"Порт {duplicate.Key} выбран несколько раз.";
        selectionHint.Text = hint;
        selectionHint.Visibility = hint.Length == 0 ? Visibility.Collapsed : Visibility.Visible;
        flash_all.IsEnabled = selected.Length > 0 && hint.Length == 0;
    }

    private void Workspace_SizeChanged(object sender, SizeChangedEventArgs e)
    {
        bool compact = e.NewSize.Width < 940;
        railColumn.Width = new GridLength(compact ? 0 : 318);
        Grid.SetColumn(rail, compact ? 0 : 1);
        Grid.SetRow(rail, compact ? 1 : 0);
        rail.Margin = compact ? new Thickness(0, 14, 0, 0) : new Thickness(16, 0, 0, 0);
    }

    private void RowMenu_Click(object sender, RoutedEventArgs e)
    {
        var button = (Button)sender;
        button.ContextMenu.PlacementTarget = button;
        button.ContextMenu.Placement = PlacementMode.Bottom;
        button.ContextMenu.IsOpen = true;
    }

    private void Update_Click(object sender, RoutedEventArgs e)
    {
        CloseMonitor();
        RefreshCatalog();
    }

    private void Clear_Click(object sender, RoutedEventArgs e)
    {
        if (MessageBox.Show("Очистить выбор прошивок и COM-портов?", Title, MessageBoxButton.YesNo) != MessageBoxResult.Yes)
            return;
        CloseMonitor();
        txt_err.Clear();
        rows.Clear();
        for (int i = 0; i < BoardCount; i++) rows.Add(new FlashInfo());
        chk_all.IsChecked = false;
        RefreshCatalog();
    }

    private void chk_all_Click(object sender, RoutedEventArgs e)
    {
        bool select = rows.Any(row => !row.chk);
        foreach (var row in rows) row.chk = select;
    }

    private async void cmdFlash_Clicked(object sender, RoutedEventArgs e) =>
        await FlashAsync(new[] { (FlashInfo)((FrameworkElement)sender).DataContext });

    private async void flash_all_Click(object sender, RoutedEventArgs e) =>
        await FlashAsync(rows.Where(row => row.chk).ToArray());

    private async Task FlashAsync(FlashInfo[] selected)
    {
        if (flashing) return;
        flashing = true;
        queuePanel.IsEnabled = writeOptions.IsEnabled = Update.IsEnabled = false;
        try
        {
            var ports = SerialPort.GetPortNames();
            var jobs = new List<(FlashInfo Row, ProcessStartInfo Command)>();
            if (selected.Length == 0) throw new InvalidOperationException("Отметьте строки для прошивки.");
            foreach (var row in selected)
            {
                var profile = row.DirectoryProject.FirstOrDefault(p => p.NameDir == row.Path);
                if (profile == null || !ports.Contains(row.Port))
                    throw new InvalidOperationException($"Строка {row.elem}: выберите прошивку и подключённый COM-порт.");
                if (jobs.Any(j => j.Row.Port == row.Port))
                    throw new InvalidOperationException($"Порт {row.Port} выбран несколько раз.");
                jobs.Add((row, profile.CreateFlashCommand(AppContext.BaseDirectory, row.Port, chk_erase.IsChecked == true)));
            }

            CloseMonitor();
            txt_err.Clear();
            foreach (var (row, command) in jobs)
            {
                row.SetStage(FlashStage.Writing);
                logSource.Text = $"Запись · {row.Path} · {row.Port}";
                row.flMAC = null;
                txt_log.Clear();
                try
                {
                    var exitCode = await RunTool(row, command);
                    row.SetStage(exitCode == 0 ? FlashStage.Success : FlashStage.Failure, $"Ошибка esptool ({exitCode})");
                    logSource.Text = $"{row.Port} · {row.DisplayStatus}";
                    if (chk_noClose.IsChecked == true)
                    {
                        try { OpenLog(row.Port); }
                        catch (Exception ex) { txt_log.AppendText($"{Environment.NewLine}Журнал: {ex.Message}"); }
                    }
                    if (exitCode == 0 && row == selected[^1] && chk_erase_Copy.IsChecked == true)
                        OpenMonitor(row.Port);
                }
                catch (Exception e)
                {
                    row.SetStage(FlashStage.Failure, "Ошибка: " + e.Message);
                    logSource.Text = $"Ошибка записи · {row.Port}";
                    txt_log.AppendText(Environment.NewLine + e.Message);
                }
            }
        }
        catch (Exception e) { MessageBox.Show(e.Message, Title); }
        finally
        {
            flashing = false;
            queuePanel.IsEnabled = writeOptions.IsEnabled = Update.IsEnabled = true;
        }
    }

    private static string LogPath(string port) => Path.Combine(Path.GetTempPath(), $"espMACi{port}.txt");

    private async Task<int> RunTool(FlashInfo row, ProcessStartInfo command)
    {
        using var log = File.CreateText(LogPath(row.Port));
        using var process = Process.Start(command) ?? throw new IOException("Не удалось запустить esptool.");
        async Task ReadOutput(StreamReader stream)
        {
            while (await stream.ReadLineAsync() is string line)
            {
                log.WriteLine(line);
                log.Flush();
                txt_log.AppendText(line + Environment.NewLine);
                txt_log.ScrollToEnd();
                if (line.StartsWith("MAC: ", StringComparison.Ordinal))
                    row.flMAC = line[5..].Replace(":", "").Trim();
            }
        }
        await Task.WhenAll(ReadOutput(process.StandardOutput), ReadOutput(process.StandardError), process.WaitForExitAsync());
        return process.ExitCode;
    }

    private static void OpenLog(string port) =>
        Process.Start(new ProcessStartInfo(LogPath(port)) { UseShellExecute = true });

    private void cmdLog_Clicked(object sender, RoutedEventArgs e)
    {
        var row = (FlashInfo)((FrameworkElement)sender).DataContext;
        try
        {
            if (string.IsNullOrEmpty(row.Port) || !File.Exists(LogPath(row.Port)))
                throw new FileNotFoundException("Для выбранного порта ещё нет журнала.");
            OpenLog(row.Port);
        }
        catch (Exception ex) { MessageBox.Show(ex.Message, Title); }
    }

    private void OpenMonitor(string port)
    {
        CloseMonitor();
        try
        {
            monitor = new SerialPort(port, 115200);
            monitor.DataReceived += DataReceived;
            monitor.Open();
            btn_Close_COM.Visibility = Visibility.Visible;
            logSource.Text = $"Монитор · {port} · 115200 бод";
            txt_log.Text = $"{port} · 115200{Environment.NewLine}";
            txt_err.Clear();
        }
        catch (Exception e)
        {
            CloseMonitor();
            txt_log.AppendText($"{Environment.NewLine}Монитор: {e.Message}");
        }
    }

    private static readonly string[] TestErrors =
    {
        "ccs co2 levie dannie", "ccs co2 net dannih", "ccs co2 ne viden na shine",
        "sht20 does not show hum", "sht20 does not show temp", "illuminat does not show", "no wifi"
    };

    private void DataReceived(object sender, SerialDataReceivedEventArgs e)
    {
        var port = (SerialPort)sender;
        string data;
        try { data = port.ReadExisting(); }
        catch (InvalidOperationException) { return; }
        catch (IOException) { return; }
        Dispatcher.InvokeAsync(() =>
        {
            if (monitor != port) return;
            txt_log.AppendText(data);
            txt_log.ScrollToEnd();
            var buffer = serialTail + data;
            var lines = buffer.Split('\n');
            serialTail = lines[^1];
            if (serialTail.Length > 4096) serialTail = serialTail[^4096..];
            foreach (var line in lines.SkipLast(1))
            {
                if (line.Trim() == "succes")
                {
                    txt_err.Text = "Успех";
                    txt_err.Foreground = Brushes.Green;
                }
                else if (TestErrors.Any(s => line.Contains(s, StringComparison.Ordinal)))
                {
                    txt_err.AppendText(Environment.NewLine + line.Trim());
                    txt_err.Foreground = Brushes.Red;
                }
            }
        });
    }

    private void CloseMonitor()
    {
        if (monitor == null) return;
        var previous = monitor;
        monitor = null;
        btn_Close_COM.Visibility = Visibility.Collapsed;
        logSource.Text = "Монитор остановлен";
        previous.DataReceived -= DataReceived;
        previous.Dispose();
        serialTail = "";
    }

    private void cmdMonitor_Clicked(object sender, RoutedEventArgs e)
    {
        var row = (FlashInfo)((FrameworkElement)sender).DataContext;
        if (string.IsNullOrEmpty(row.Port)) MessageBox.Show("Выберите COM-порт.", Title);
        else OpenMonitor(row.Port);
    }

    private void btn_Close_COM_Click(object sender, RoutedEventArgs e)
    {
        CloseMonitor();
        txt_log.AppendText(Environment.NewLine + "Монитор остановлен.");
    }

    private void cmd_Settings_Clicked(object sender, RoutedEventArgs e)
    {
        var row = (FlashInfo)((FrameworkElement)sender).DataContext;
        if (string.IsNullOrEmpty(row.flMAC)) MessageBox.Show("Сначала прошейте плату, чтобы прочитать MAC.", Title);
        else new Settings(row.elem - 1) { Title = $"Настройки · {row.flMAC}", Owner = this }.ShowDialog();
    }

    protected override void OnClosing(CancelEventArgs e)
    {
        base.OnClosing(e);
        if (!flashing) return;
        e.Cancel = true;
        MessageBox.Show("Дождитесь завершения прошивки.", Title);
    }

    private void Window_Closed(object sender, EventArgs e)
    {
        CloseMonitor();
        settings.spifflash = JsonSerializer.Serialize(rows);
        settings.erase = chk_erase.IsChecked == true;
        settings.noClosing = chk_noClose.IsChecked == true;
        settings.monitoe = chk_erase_Copy.IsChecked == true;
        settings.Save();
    }
}
