using System.Windows;

namespace WpfApp1;

public partial class Retrotop : Window
{
    private readonly DeviceIOT device;

    public Retrotop(DeviceIOT device)
    {
        InitializeComponent();
        this.device = device;
        txt_numPIN.Text = device.numPIN.ToString();
        txt_numBUTTON.Text = device.numButton.ToString();
        txt_numPIN2.Text = device.numPIN2.ToString();
        txt_numBUTTON2.Text = device.numButton2.ToString();
        chk_mechanicBtn.IsChecked = device.MechanicBtn;
    }

    private void btn_Save_Click(object sender, RoutedEventArgs e)
    {
        if (!int.TryParse(txt_numPIN.Text, out var numPIN) || !DeviceIOT.ValidGpio(numPIN, true) ||
            !int.TryParse(txt_numBUTTON.Text, out var numButton) || !DeviceIOT.ValidGpio(numButton, false) ||
            !int.TryParse(txt_numPIN2.Text, out var numPIN2) || !DeviceIOT.ValidGpio(numPIN2, true) ||
            !int.TryParse(txt_numBUTTON2.Text, out var numButton2) || !DeviceIOT.ValidGpio(numButton2, false))
        {
            MessageBox.Show(this, "Проверьте GPIO выходов и кнопок. GPIO 34–39 доступны только для входа.", Title);
            return;
        }
        device.numPIN = numPIN;
        device.numButton = numButton;
        device.numPIN2 = numPIN2;
        device.numButton2 = numButton2;
        device.MechanicBtn = chk_mechanicBtn.IsChecked == true;
        Close();
    }
}
