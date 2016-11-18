using System;
using System.Collections.Generic;
using System.IO;
using System.Security.Cryptography;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using Newtonsoft.Json;

namespace AgentsMovingVis
{
    /// <summary>
    /// This is the main type for your game.
    /// </summary>
    public class Game1 : Game
    {
        GraphicsDeviceManager graphics;
        SpriteBatch spriteBatch;
        private List<Iteration> simulationData;
        //     private List<Obstacle> obstacles;
        double timeFromLastUpdate = 0;
        double timeToAgentStep = 100;
        private Vector2 offset;
        private int previousScrollValue;

        private Texture2D agentTexture2D;
        private SpriteFont Font;
        private int currentIteration = 0;
        private float zoom = 3f;

        private bool IsSpaceDragModeOn;
        private Point previousMouseLBPressedPoint;

        public Game1()
        {
            graphics = new GraphicsDeviceManager(this);
            graphics.PreferredBackBufferHeight = 600;
            graphics.PreferredBackBufferWidth = 800;
            Content.RootDirectory = "Content";
        }

        /// <summary>
        /// Allows the game to perform any initialization it needs to before starting to run.
        /// This is where it can query for any required services and load any non-graphic
        /// related content.  Calling base.Initialize will enumerate through any components
        /// and initialize them as well.
        /// </summary>
        protected override void Initialize()
        {
            // TODO: Add your initialization logic here
            IsMouseVisible = true;
            Window.AllowUserResizing = true;
            Window.ClientSizeChanged += new EventHandler<EventArgs>(Window_ClientSizeChanged);

            string path = @"simData.txt";
            string json = "";
            json = File.ReadAllText(path);
            simulationData = JsonConvert.DeserializeObject<List<Iteration>>(json);

            base.Initialize();
        }

        /// <summary>
        /// LoadContent will be called once per game and is the place to load
        /// all of your content.
        /// </summary>
        protected override void LoadContent()
        {
            // Create a new SpriteBatch, which can be used to draw textures.
            spriteBatch = new SpriteBatch(GraphicsDevice);
            agentTexture2D = Content.Load<Texture2D>("agent");
            Font = Content.Load<SpriteFont>("Arial");
            // TODO: use this.Content to load your game content here
        }

        /// <summary>
        /// UnloadContent will be called once per game and is the place to unload
        /// game-specific content.
        /// </summary>
        protected override void UnloadContent()
        {
            agentTexture2D.Dispose();
            // TODO: Unload any non ContentManager content here
        }

        /// <summary>
        /// Allows the game to run logic such as updating the world,
        /// checking for collisions, gathering input, and playing audio.
        /// </summary>
        /// <param name="gameTime">Provides a snapshot of timing values.</param>
        protected override void Update(GameTime gameTime)
        {
            try
            {
                if (GamePad.GetState(PlayerIndex.One).Buttons.Back == ButtonState.Pressed || Keyboard.GetState().IsKeyDown(Keys.Escape))
                    Exit();

                var mouseState = Mouse.GetState();
                var keyBoardState = Keyboard.GetState();

                //Wheel rotating tracking
                int currentWheelValue = mouseState.ScrollWheelValue;
                if (currentWheelValue > previousScrollValue)
                {
                    zoom += zoom <= 10 ? 0.05f : 0;
                }
                else
                {
                    if (currentWheelValue < previousScrollValue)
                    {
                        zoom -= zoom >= -10 ? 0.05f : 0;
                    }
                }
                previousScrollValue = Mouse.GetState().ScrollWheelValue;

                //track draging the space
                if (!IsSpaceDragModeOn && mouseState.LeftButton == ButtonState.Pressed)
                {
                    previousMouseLBPressedPoint = mouseState.Position;
                    IsSpaceDragModeOn = true;
                    //MessagesStack.PutMessage("MLB pressed and dragg mode on", 2);
                }

                if (IsSpaceDragModeOn && mouseState.LeftButton == ButtonState.Released)
                {
                    IsSpaceDragModeOn = false;
                    //MessagesStack.PutMessage("MLB released and dragg mode off", 2);
                }

                if (keyBoardState.IsKeyDown(Keys.Up))
                {
                    offset.Y--;
                }

                if (keyBoardState.IsKeyDown(Keys.Down))
                {
                    offset.Y++;
                }

                if (keyBoardState.IsKeyDown(Keys.Left))
                {
                    offset.X--;
                }

                if (keyBoardState.IsKeyDown(Keys.Right))
                {
                    offset.X++;
                }

                if (IsSpaceDragModeOn)
                {
                    //MessagesStack.PutMessage("Dragg mode on", 3);
                    Point curentLBPressedPoint = mouseState.Position;
                    offset += (curentLBPressedPoint - previousMouseLBPressedPoint).ToVector2();
                    previousMouseLBPressedPoint = curentLBPressedPoint;

                    //double dist = Math.Sqrt(
                    //        Math.Pow(previousMouseLBPressedPoint.X - curentLBPressedPoint.X, 2) +
                    //        Math.Pow(previousMouseLBPressedPoint.Y - curentLBPressedPoint.Y, 2));
                    //if (dist >= 5){ }
                }

                timeFromLastUpdate += gameTime.ElapsedGameTime.TotalMilliseconds;

                if (timeFromLastUpdate >= timeToAgentStep) //timeConstraint
                {
                    if (currentIteration < simulationData.Count)
                    {
                        currentIteration++;
                    }

                    timeFromLastUpdate = 0;
                }


                // TODO: Add your update logic here

                base.Update(gameTime);
            }
            catch (Exception ex)
            {
                File.WriteAllText("Error.txt", ex.ToString());
            }
        }

        /// <summary>
        /// This is called when the game should draw itself.
        /// </summary>
        /// <param name="gameTime">Provides a snapshot of timing values.</param>
        protected override void Draw(GameTime gameTime)
        {

            spriteBatch.Begin();
            try
            {
                GraphicsDevice.Clear(Color.LightGray);

                for (int i = 0; i < simulationData[currentIteration].agents.Count; i++)
                {
                    float x = Convert.ToSingle(simulationData[currentIteration].agents[i].X.Replace('.', ','));
                    float y = Convert.ToSingle(simulationData[currentIteration].agents[i].Y.Replace('.', ','));
                    spriteBatch.Draw(agentTexture2D, new Vector2(offset.X + (x * zoom) - agentTexture2D.Width /2, offset.Y + (y * zoom) - agentTexture2D.Width/2), null, Color.White, 0.0f, new Vector2(0, 0), new Vector2(0.05f, 0.05f), SpriteEffects.None, 0);
                }

                spriteBatch.DrawString(Font, "zoom: " + zoom.ToString(), new Vector2(50, 50), Color.Red, 0f, Vector2.Zero, new Vector2(1, 1), SpriteEffects.None, 0);
                spriteBatch.DrawString(Font, "Iteration: " + currentIteration.ToString(), new Vector2(50, 100), Color.Red, 0f, Vector2.Zero, new Vector2(1, 1), SpriteEffects.None, 0);
            }
            catch (Exception ex)
            {
                File.WriteAllText("Error.txt", ex.ToString());
            }
            spriteBatch.End();
            base.Draw(gameTime);
        }

        void Window_ClientSizeChanged(object sender, EventArgs e)
        {
            graphics.PreferredBackBufferWidth = Window.ClientBounds.Width;
            graphics.PreferredBackBufferHeight = Window.ClientBounds.Height;
        }
    }
}
