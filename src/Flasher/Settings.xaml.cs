using System;
using System.IO;
using System.Windows;
using System.Windows.Media.Imaging;

namespace WpfApp1;

public partial class Settings : Window
{
    private readonly LocalPreset preset;

    internal Settings(string profile, string mac, string directory)
    {
        InitializeComponent();
        preset = new LocalPreset(profile, mac, directory);
        Title = $"{profile} · {mac}";
        model.Text = Title;
        var image = Path.Combine(AppContext.BaseDirectory, "img", profile == "SmartKitchen" ? "smartkitchen_1.jpg" : "smartkitchen_2.jpg");
        if (File.Exists(image)) picture.Source = new BitmapImage(new Uri(image));
        lb_settings.ItemsSource = preset.Devices;
    }

    private void cmd_Edit_Clicked(object sender, RoutedEventArgs e)
    {
        var device = (DeviceIOT)((FrameworkElement)sender).DataContext;
        Window editor = device.Type switch
        {
            "svet" => new Svet(device),
            "retrotop_up" => new Retrotop(device),
            _ => null
        };
        if (editor == null) return;
        editor.Owner = this;
        editor.ShowDialog();
        device.Describe();
    }

    private void btn_Post_Click(object sender, RoutedEventArgs e)
    {
        try { preset.Save(); }
        catch (Exception ex) { MessageBox.Show(this, ex.Message, Title); return; }
        DialogResult = true;
    }
}
