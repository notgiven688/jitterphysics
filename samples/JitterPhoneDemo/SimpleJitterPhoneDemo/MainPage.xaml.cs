using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Net;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Navigation;
using Microsoft.Phone.Controls;
using Microsoft.Phone.Shell;
using Microsoft.Xna.Framework;
using MonoGame.Framework.WindowsPhone;

namespace SimpleJitterPhoneDemo
{
    public partial class MainPage : PhoneApplicationPage
    {
        private DemoGame game;

        public MainPage()
        {
            InitializeComponent();

            game = XamlGame<DemoGame>.Create("", this);
        }

        protected override void OnBackKeyPress(CancelEventArgs e)
        {
            game.Exit();

            base.OnBackKeyPress(e);
        }
    }
}