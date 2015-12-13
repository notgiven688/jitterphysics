using Foundation;
using Microsoft.Xna.Framework;
using UIKit;

using JitterSample;

namespace JitteriOSSample
{
    [Register("AppDelegate")]
    public class AppDelegate : UIApplicationDelegate
    {
        public override UIWindow Window { get; set; }

        public override void FinishedLaunching(UIApplication application)
        {
            var rect = UIScreen.MainScreen.Bounds;
            JitterPhysicsGame.PreferredSize = new Vector2((float)rect.Width, (float)rect.Height);

            var game = new JitterPhysicsGame();
            game.Run();
        }
    }
}
