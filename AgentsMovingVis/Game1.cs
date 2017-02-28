using System;
using System.Collections.Generic;
using System.IO;
using System.Net.Configuration;
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
        private Dictionary<int, Dictionary<long, Agent>> simDataDictionary;
        private List<Obstacle> obstacles;
        private List<Area> subareas;

        //     private List<Obstacle> obstacles;
        double timeFromLastUpdate = 0;
        double timeToAgentStep = 300;
        private Vector2 offset;
        private int previousScrollValue;

        private Texture2D onePixelTexture;
        private Texture2D agentTexture2D;
        //private Texture2D agentTexture2DRed;
        private SpriteFont Font;
        private int currentIteration = 0;
        private bool IsPause = false;
        private float zoom = 3f;
        KeyboardState oldState;

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

            string simDataPath = @"simData.txt";
            string json = "";
            //json = File.ReadAllText(simDataPath);
            //simulationData = JsonConvert.DeserializeObject<List<Iteration>>(json);

            //simDataDictionary = new Dictionary<int, Dictionary<long, Agent>>();
            //for (int i = 0; i < simulationData.Count; i++)
            //{
            //    Dictionary<long, Agent> agentsOnIteration = new Dictionary<long, Agent>();
            //    for (int j = 0; j < simulationData[i].agents.Count; j++)
            //    {
            //        agentsOnIteration[simulationData[i].agents[j].agentID] = simulationData[i].agents[j];
            //    }
            //    simDataDictionary[simulationData[i].iteration] = agentsOnIteration;
            //}

            simDataDictionary = new Dictionary<int, Dictionary<long, Agent>>();
            string line = "";
            StreamReader file = new StreamReader(@"testsimData.txt");
            while ((line = file.ReadLine()) != null)
            {
                int iterNum = Convert.ToInt32(line);

                Dictionary<long, Agent> agentsOnIteration = new Dictionary<long, Agent>();

                line = file.ReadLine();
                int agentsNum = Convert.ToInt32(line);
                for (int i = 0; i < agentsNum; i++)
                {
                    line = file.ReadLine();
                    int isDeleted = Convert.ToInt32(line);

                    line = file.ReadLine();
                    int agentId = Convert.ToInt32(line);

                    line = file.ReadLine();
                    float coordX = Convert.ToSingle(line.Replace('.', ','));

                    line = file.ReadLine();
                    float coordY = Convert.ToSingle(line.Replace('.', ','));

                    Agent agent = new Agent();
                    agent.X = coordX;
                    agent.Y = coordY;
                    agent.agentID = agentId;

                    agentsOnIteration[agentId] = agent;
                }
                simDataDictionary[iterNum] = agentsOnIteration;
            }

            file.Close();

            simulationData = new List<Iteration>();
            foreach (var v in simDataDictionary)
            {
                var iteration = new Iteration();
                iteration.iteration = v.Key;
                iteration.agents = new List<Agent>();
                foreach (var agent in v.Value)
                {
                    iteration.agents.Add(agent.Value);
                }
                simulationData.Add(iteration);
            }


            string obstPath = @"test_obstacles.txt";
            json = File.ReadAllText(obstPath);
            obstacles = JsonConvert.DeserializeObject<List<Obstacle>>(json);

            string subareasPath = @"test_areas.txt";
            json = File.ReadAllText(subareasPath);
            subareas = JsonConvert.DeserializeObject<List<Area>>(json);

            oldState = Keyboard.GetState();

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
            //agentTexture2DRed = Content.Load<Texture2D>("agent");
            Font = Content.Load<SpriteFont>("Arial");
            onePixelTexture = CreateTexture2D(Color.Aquamarine, 1, 1);

            // TODO: use this.Content to load your game content here
        }

        /// <summary>
        /// UnloadContent will be called once per game and is the place to unload
        /// game-specific content.
        /// </summary>
        protected override void UnloadContent()
        {
            agentTexture2D.Dispose();
            onePixelTexture.Dispose();
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
                if (GamePad.GetState(PlayerIndex.One).Buttons.Back == ButtonState.Pressed ||
                    Keyboard.GetState().IsKeyDown(Keys.Escape))
                    Exit();

                var mouseState = Mouse.GetState();
                var keyBoardState = Keyboard.GetState();

                //Wheel rotating tracking
                int currentWheelValue = mouseState.ScrollWheelValue;
                if (currentWheelValue > previousScrollValue)
                {
                    zoom += zoom <= 10 ? 0.1f : 0;
                }
                else
                {
                    if (currentWheelValue < previousScrollValue)
                    {
                        zoom -= zoom >= -10 ? 0.1f : 0;
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

                if (keyBoardState.IsKeyDown(Keys.Space))
                {
                    currentIteration = 0;
                }

                if (keyBoardState.IsKeyDown(Keys.P))
                {
                    IsPause = true;
                }

                if (keyBoardState.IsKeyUp(Keys.P))
                {
                    IsPause = false;
                }

                if (keyBoardState.IsKeyDown(Keys.OemMinus) && !oldState.IsKeyDown(Keys.OemMinus))
                {
                    if (currentIteration > 0)
                    {
                        currentIteration--;
                    }
                }

                if (keyBoardState.IsKeyDown(Keys.OemPlus) && !oldState.IsKeyDown(Keys.OemPlus))
                {
                    if (currentIteration < simulationData.Count - 1)
                    {
                        currentIteration++;
                    }
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
                    if (currentIteration < simulationData.Count - 1 && !IsPause)
                    {
                        currentIteration++;
                    }

                    timeFromLastUpdate = 0;
                }


                // Update saved state.
                oldState = keyBoardState;


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
            //try
            //{
            GraphicsDevice.Clear(Color.Black);

            try
            {
                ////drawing subareas
                for (int i = 0; i < subareas.Count; i++)
                {
                    float x1 = (subareas[i].x1*zoom) + offset.X;
                    float y1 = (subareas[i].y1*zoom) + offset.Y;
                    float x1_ = ((subareas[i].x1 + subareas[i].horizontalIndent)*zoom) + offset.X;
                    float y1_ = ((subareas[i].y1 + subareas[i].verticalIndent)*zoom) + offset.Y;

                    float x2 = (subareas[i].x2*zoom) + offset.X;
                    float y2 = (subareas[i].y2*zoom) + offset.Y;
                    float x2_ = ((subareas[i].x2 - subareas[i].horizontalIndent)*zoom) + offset.X;
                    float y2_ = ((subareas[i].y2 - subareas[i].verticalIndent)*zoom) + offset.Y;

                    Rectangle RectForBorders = new Rectangle((int) x1, (int) y1, (int) (x2 - x1), (int) (y2 - y1));
                    spriteBatch.Draw(onePixelTexture, RectForBorders, null, Color.Azure);

                    Rectangle centralRect = new Rectangle((int) x1_, (int) y1_, (int) (x2_ - x1_), (int) (y2_ - y1_));
                    spriteBatch.Draw(onePixelTexture, centralRect, null, Color.Black);

                    DrawLine(spriteBatch, new Vector2(x1, y1), new Vector2(x1, y2), Color.Yellow, 1);
                    DrawLine(spriteBatch, new Vector2(x1, y2), new Vector2(x2, y2), Color.Yellow, 1);
                    DrawLine(spriteBatch, new Vector2(x2, y2), new Vector2(x2, y1), Color.Yellow, 1);
                    DrawLine(spriteBatch, new Vector2(x2, y1), new Vector2(x1, y1), Color.Yellow, 1);
                }
            }
            catch (Exception ex)
            {
                File.WriteAllText("Error.txt", ex.ToString());
            }


            try
            {
                ////drawing obstacles
                for (int i = 0; i < obstacles.Count; i++)
                {
                    for (int j = 0; j < obstacles[i].points.Count - 1; j++)
                    {
                        float x1 = (obstacles[i].points[j].X*zoom) + offset.X;
                        float y1 = (obstacles[i].points[j].Y*zoom) + offset.Y;

                        float x2 = (obstacles[i].points[j + 1].X*zoom) + offset.X;
                        float y2 = (obstacles[i].points[j + 1].Y*zoom) + offset.Y;

                        DrawLine(spriteBatch, new Vector2(x1, y1), new Vector2(x2, y2), Color.Blue, 2);
                    }
                }
            }
            catch (Exception ex)
            {
                File.WriteAllText("Error.txt", ex.ToString());
            }


            try
            {
                //drawing agents
                for (int i = 0; i < simulationData[currentIteration].agents.Count; i++)
                {
                    float x = 0;
                    float y = 0;
                    try
                    {
                        x = simulationData[currentIteration].agents[i].X;
                        y = simulationData[currentIteration].agents[i].Y;
                    }
                    catch (Exception)
                    {
                        throw;
                    }

                    if (i < simulationData[currentIteration].agents.Count / 2)
                    {
                        spriteBatch.Draw(agentTexture2D, new Vector2(offset.X + (x * zoom) - (agentTexture2D.Width * 0.03f / 2), offset.Y + (y * zoom) - (agentTexture2D.Height * 0.03f / 2)), null, Color.White, 0.0f, new Vector2(0, 0), new Vector2(0.03f, 0.03f), SpriteEffects.None, 0);
                    }
                    else
                    {
                        spriteBatch.Draw(agentTexture2D, new Vector2(offset.X + (x * zoom) - (agentTexture2D.Width * 0.03f / 2), offset.Y + (y * zoom) - (agentTexture2D.Height * 0.03f / 2)), null, Color.Red, 0.0f, new Vector2(0, 0), new Vector2(0.03f, 0.03f), SpriteEffects.None, 0);
                    }
                }
            }
            catch (Exception ex)
            {
                File.WriteAllText("Error.txt", ex.ToString());
            }

            try
            {
                //drawing trace
                for (int i = 0; i < simulationData[currentIteration].agents.Count; i++)
                {
                    float x1 = simulationData[currentIteration].agents[i].X;
                    float y1 = simulationData[currentIteration].agents[i].Y;

                    if (currentIteration > 0)
                    {
                        float x2 = simDataDictionary[currentIteration - 1][simulationData[currentIteration].agents[i].agentID].X; // simulationData[currentIteration - 1].agents[i].X;
                        float y2 = simDataDictionary[currentIteration - 1][simulationData[currentIteration].agents[i].agentID].Y; //simulationData[currentIteration - 1].agents[i].Y;

                        DrawLine(spriteBatch, new Vector2(offset.X + (x1 * zoom), offset.Y + (y1 * zoom)), new Vector2(offset.X + (x2 * zoom), offset.Y + (y2 * zoom)), Color.Green, 1);
                    }
                }

                spriteBatch.DrawString(Font, "zoom: " + zoom.ToString(), new Vector2(50, 50), Color.Red, 0f, Vector2.Zero, new Vector2(1, 1), SpriteEffects.None, 0);
                spriteBatch.DrawString(Font, "Iteration: " + currentIteration.ToString(), new Vector2(50, 100), Color.Red, 0f, Vector2.Zero, new Vector2(1, 1), SpriteEffects.None, 0);
            }
            catch (Exception ex)
            {
                File.WriteAllText("Error.txt", ex.ToString());
            }

            //}
            //catch (Exception ex)
            //{
            //    File.WriteAllText("Error.txt", ex.ToString());
            //}
            spriteBatch.End();
            base.Draw(gameTime);
        }

        void Window_ClientSizeChanged(object sender, EventArgs e)
        {
            graphics.PreferredBackBufferWidth = Window.ClientBounds.Width;
            graphics.PreferredBackBufferHeight = Window.ClientBounds.Height;
        }
        public void DrawLine(SpriteBatch spriteBatch, Vector2 begin, Vector2 end, Color color, int width = 1)
        {
            Rectangle r = new Rectangle((int)(begin.X - width / 2.0), (int)(begin.Y - width / 2.0), (int)(end - begin).Length(), width);
            Vector2 v = Vector2.Normalize(begin - end);
            float angle = (float)Math.Acos(Vector2.Dot(v, -Vector2.UnitX));
            if (begin.Y > end.Y) angle = MathHelper.TwoPi - angle;

            spriteBatch.Draw(onePixelTexture, r, null, color, angle, Vector2.Zero, SpriteEffects.None, 0);
        }

        public Texture2D CreateTexture2D(Color color, int h = 1, int w = 1)
        {
            Texture2D texture = new Texture2D(GraphicsDevice, h, w, false, SurfaceFormat.Color);
            texture.SetData<Color>(new Color[] { color });
            return texture;
        }
    }


}
