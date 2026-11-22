using System.Diagnostics;
using System.Globalization;
using System.Text;
using System.Drawing;
using System.Windows.Forms;

namespace MechanicsMinimal;

static class Program
{
    [STAThread]
    static void Main()
    {
        Application.EnableVisualStyles();
        Application.SetCompatibleTextRenderingDefault(false);
        Application.Run(new MainForm());
    }
}

class Body
{
    public double X, Y, W, H, Vx, Vy, OldX, OldY;
    public bool Fixed, Fast, Target;
    public double InvMass => Fixed ? 0 : Fast ? 2 : 1;

    public Body(double x, double y, double w, double h,
                double vx = 0, double vy = 0, bool fixedBody = false,
                bool fast = false, bool target = false)
    {
        X = x; Y = y; W = w; H = h; Vx = vx; Vy = vy;
        Fixed = fixedBody; Fast = fast; Target = target;
    }
}

class Physics
{
    public const int NormalHz = 60;
    public const int FastHz = 240;
    public const double ProjectileSpeed = 2000;
    public const double Restitution = 0.8;

    public readonly List<Body> Fixed = new();
    public readonly List<Body> Normal = new();
    public readonly List<Body> Fast = new();
    public readonly List<Body> All = new();

    public int Scene, Mode, Collisions, Missed, CcdLimitHits, FastSteps;
    public double MaxPenetration;
    public bool Hit, Passed;
    public bool Done => Scene == 0 && (Hit || Passed);

    public Physics(int scene, int mode)
    {
        Scene = scene; Mode = mode;
        if (scene == 0)
        {
            Fixed.Add(new Body(470, 260, 10, 100, fixedBody: true, target: true));
            Fast.Add(new Body(60, 260, 8, 8, ProjectileSpeed, fast: true));
        }
        else
        {
            Fixed.Add(new Body(350, 260, 40, 40, fixedBody: true));
            Fixed.Add(new Body(480, 260, 40, 40, fixedBody: true));
            Fixed.Add(new Body(610, 260, 40, 40, fixedBody: true));
            for (int i = 0; i < 10; i++)
                Normal.Add(new Body(305 + i % 5 * 78, 225 + i / 5 * 70,
                    20, 20, i % 2 == 0 ? 135 : -125, i < 5 ? 40 : -40));
            Fast.Add(new Body(300, 270, 8, 8, 800, -420, fast: true));
        }
        All.AddRange(Fixed);
        All.AddRange(Normal);
        All.AddRange(Fast);
    }

    static bool Overlap(Body a, Body b, out double nx, out double ny, out double depth)
    {
        double px = (a.W + b.W) / 2 - Math.Abs(a.X - b.X);
        double py = (a.H + b.H) / 2 - Math.Abs(a.Y - b.Y);
        nx = ny = depth = 0;
        if (px <= 0 || py <= 0) return false;
        if (px < py) { nx = a.X < b.X ? 1 : -1; depth = px; }
        else { ny = a.Y < b.Y ? 1 : -1; depth = py; }
        return true;
    }

    void Impulse(Body a, Body b, double nx, double ny)
    {
        double closing = (b.Vx - a.Vx) * nx + (b.Vy - a.Vy) * ny;
        if (closing >= 0) return;
        double j = -(1 + Restitution) * closing / (a.InvMass + b.InvMass);
        a.Vx -= j * nx * a.InvMass;
        a.Vy -= j * ny * a.InvMass;
        b.Vx += j * nx * b.InvMass;
        b.Vy += j * ny * b.InvMass;
        Collisions++;
        if ((a.Target && b.Fast) || (b.Target && a.Fast)) Hit = true;
    }

    void Separate(Body a, Body b, double nx, double ny, double amount)
    {
        double sum = a.InvMass + b.InvMass;
        if (sum == 0) return;
        a.X -= nx * amount * a.InvMass / sum;
        a.Y -= ny * amount * a.InvMass / sum;
        b.X += nx * amount * b.InvMass / sum;
        b.Y += ny * amount * b.InvMass / sum;
    }

    void Solve(bool fastOnly = false, bool normalOnly = false)
    {
        for (int k = 0; k < 4; k++)
            for (int i = 0; i < All.Count; i++)
                for (int j = i + 1; j < All.Count; j++)
                {
                    Body a = All[i], b = All[j];
                    if (a.Fixed && b.Fixed) continue;
                    if (fastOnly && !a.Fast && !b.Fast) continue;
                    if (normalOnly && (a.Fast || b.Fast)) continue;
                    if (!Overlap(a, b, out double nx, out double ny, out double d)) continue;
                    MaxPenetration = Math.Max(MaxPenetration, d);
                    Separate(a, b, nx, ny, d + 0.0001);
                    Impulse(a, b, nx, ny);
                }
    }

    static bool Sweep(Body a, Body b, double dt,
                      out double time, out double nx, out double ny)
    {
        time = nx = ny = 0;
        double dx = b.X - a.X, dy = b.Y - a.Y;
        double vx = a.Vx - b.Vx, vy = a.Vy - b.Vy;
        double hx = (a.W + b.W) / 2, hy = (a.H + b.H) / 2;
        if (Math.Abs(dx) < hx && Math.Abs(dy) < hy) return false;
        double ex, lx, ey, ly;
        if (Math.Abs(vx) < 1e-10)
        {
            if (Math.Abs(dx) >= hx) return false;
            ex = double.NegativeInfinity; lx = double.PositiveInfinity;
        }
        else
        {
            double t1 = (dx - hx) / vx, t2 = (dx + hx) / vx;
            ex = Math.Min(t1, t2); lx = Math.Max(t1, t2);
        }
        if (Math.Abs(vy) < 1e-10)
        {
            if (Math.Abs(dy) >= hy) return false;
            ey = double.NegativeInfinity; ly = double.PositiveInfinity;
        }
        else
        {
            double t1 = (dy - hy) / vy, t2 = (dy + hy) / vy;
            ey = Math.Min(t1, t2); ly = Math.Max(t1, t2);
        }
        double entry = Math.Max(ex, ey), exit = Math.Min(lx, ly);
        if (entry > exit || entry < -1e-9 || entry > dt || exit < 0 ||
            double.IsInfinity(entry)) return false;
        time = Math.Max(0, entry);
        if (ex >= ey) nx = Math.Sign(vx);
        else ny = Math.Sign(vy);
        return (b.Vx - a.Vx) * nx + (b.Vy - a.Vy) * ny < -1e-9;
    }

    static void Move(List<Body> bodies, double dt)
    {
        foreach (Body b in bodies)
        {
            b.OldX = b.X; b.OldY = b.Y;
            b.X += b.Vx * dt; b.Y += b.Vy * dt;
        }
    }

    void MoveAll(double dt)
    {
        foreach (Body b in All.Where(b => !b.Fixed))
        {
            b.X += b.Vx * dt; b.Y += b.Vy * dt;
        }
    }

    void CountMisses(double dt)
    {
        foreach (Body f in Fast)
            foreach (Body w in Fixed)
            {
                double x = f.X, y = f.Y;
                f.X = f.OldX; f.Y = f.OldY;
                bool crossed = Sweep(f, w, dt, out _, out _, out _);
                f.X = x; f.Y = y;
                if (crossed && !Overlap(f, w, out _, out _, out _)) Missed++;
            }
    }

    void Ccd(double dt)
    {
        Solve();
        double left = dt;
        int events = 0;
        while (left > 1e-10 && events < 64)
        {
            double first = left + 1, nx = 0, ny = 0;
            Body? aa = null, bb = null;
            for (int i = 0; i < All.Count; i++)
                for (int j = i + 1; j < All.Count; j++)
                {
                    Body a = All[i], b = All[j];
                    if (a.Fixed && b.Fixed) continue;
                    if (Sweep(a, b, left, out double t, out double x, out double y) && t < first)
                    { first = t; aa = a; bb = b; nx = x; ny = y; }
                }
            if (aa == null || bb == null) { MoveAll(left); left = 0; break; }
            MoveAll(first);
            left -= first;
            Impulse(aa, bb, nx, ny);
            Separate(aa, bb, nx, ny, 0.0001);
            events++;
        }
        if (left > 1e-10 && events >= 64) CcdLimitHits++;
        MoveAll(left);
        Solve();
    }

    void Bounds()
    {
        double left = Scene == 0 ? 12 : 280;
        double right = Scene == 0 ? 948 : 680;
        double top = Scene == 0 ? 12 : 160;
        double bottom = Scene == 0 ? 508 : 360;
        foreach (Body b in All.Where(b => !b.Fixed))
        {
            if (b.X - b.W / 2 < left) { b.X = left + b.W / 2; b.Vx = Math.Abs(b.Vx) * Restitution; }
            if (b.X + b.W / 2 > right) { b.X = right - b.W / 2; b.Vx = -Math.Abs(b.Vx) * Restitution; }
            if (b.Y - b.H / 2 < top) { b.Y = top + b.H / 2; b.Vy = Math.Abs(b.Vy) * Restitution; }
            if (b.Y + b.H / 2 > bottom) { b.Y = bottom - b.H / 2; b.Vy = -Math.Abs(b.Vy) * Restitution; }
        }
    }

    public void Step()
    {
        double dt = 1.0 / NormalHz;
        if (Mode == 1) Ccd(dt);
        else if (Mode == 0)
        {
            Move(Normal, dt); Move(Fast, dt);
            CountMisses(dt); Solve(); FastSteps++;
        }
        else
        {
            Move(Normal, dt); Solve(normalOnly: true);
            for (int i = 0; i < FastHz / NormalHz; i++)
            {
                Move(Fast, 1.0 / FastHz);
                CountMisses(1.0 / FastHz);
                Solve(fastOnly: true);
                FastSteps++;
            }
        }
        if (Mode == 1) FastSteps++;
        Bounds();
        if (Scene == 0 && !Hit && Fast[0].X > Fixed[0].X + 55) Passed = true;
    }

    public string State() => string.Join(";", All.Select(b =>
        $"{b.X:R},{b.Y:R},{b.Vx:R},{b.Vy:R}"));
}

class MainForm : Form
{
    int scene, mode;
    Physics physics = new(0, 0);
    readonly System.Windows.Forms.Timer timer = new() { Interval = 16 };
    int resultFrames;
    bool paused;

    public MainForm()
    {
        Text = "";
        ClientSize = new Size(980, 560);
        MinimumSize = new Size(700, 410);
        BackColor = Color.FromArgb(20, 24, 33);
        DoubleBuffered = true;
        KeyPreview = true;
        timer.Tick += (_, _) =>
        {
            if (!paused && !(physics.Done && scene == 0 && resultFrames++ > 7))
                physics.Step();
            Invalidate();
        };
        timer.Start();
    }

    void Reset()
    {
        physics = new Physics(scene, mode);
        resultFrames = 0;
        paused = false;
        Invalidate();
    }

    protected override bool ProcessCmdKey(ref Message msg, Keys keyData)
    {
        switch (keyData & Keys.KeyCode)
        {
            case Keys.D1: case Keys.NumPad1: mode = 0; Reset(); return true;
            case Keys.D2: case Keys.NumPad2: mode = 1; Reset(); return true;
            case Keys.D3: case Keys.NumPad3: mode = 2; Reset(); return true;
            case Keys.Tab: scene = 1 - scene; Reset(); return true;
            case Keys.R: Reset(); return true;
            case Keys.P: paused = !paused; return true;
            case Keys.M: SaveMeasurements(); return true;
        }
        return base.ProcessCmdKey(ref msg, keyData);
    }

    protected override void OnPaint(PaintEventArgs e)
    {
        base.OnPaint(e);
        var g = e.Graphics;
        g.TranslateTransform(10, 10);
        g.ScaleTransform((ClientSize.Width - 20) / 960f, (ClientSize.Height - 20) / 520f);
        var area = scene == 0
            ? new Rectangle(0, 0, 960, 520)
            : new Rectangle(270, 150, 420, 220);
        using var fill = new SolidBrush(Color.FromArgb(34, 41, 54));
        using var outline = new Pen(Color.FromArgb(103, 116, 134));
        g.FillRectangle(fill, area);
        g.DrawRectangle(outline, area);
        foreach (Body b in physics.All)
        {
            using var brush = new SolidBrush(b.Fixed
                ? Color.FromArgb(155, 164, 178)
                : b.Fast ? Color.FromArgb(255, 181, 92)
                : Color.FromArgb(91, 189, 217));
            g.FillRectangle(brush, (float)(b.X - b.W / 2), (float)(b.Y - b.H / 2), (float)b.W, (float)b.H);
        }
    }

    void SaveMeasurements()
    {
        using var dialog = new SaveFileDialog
        {
            FileName = "measurements.csv", Filter = "CSV (*.csv)|*.csv"
        };
        if (dialog.ShowDialog() != DialogResult.OK) return;
        var lines = new List<string>
        {
            "scene;mode;ms_per_frame;collisions;missed;hit;passed;fast_steps;max_penetration;ccd_limit;deterministic"
        };
        var culture = CultureInfo.InvariantCulture;
        for (int s = 0; s < 2; s++)
            for (int m = 0; m < 3; m++)
            {
                int steps = s == 0 ? 18 : 300;
                var warmup = new Physics(s, m);
                for (int i = 0; i < steps; i++) warmup.Step();
                var a = new Physics(s, m);
                double elapsed = 0;
                for (int repeat = 0; repeat < 30; repeat++)
                {
                    var run = repeat == 0 ? a : new Physics(s, m);
                    var clock = Stopwatch.StartNew();
                    for (int i = 0; i < steps; i++) run.Step();
                    clock.Stop();
                    elapsed += clock.Elapsed.TotalMilliseconds;
                }
                var b = new Physics(s, m);
                for (int i = 0; i < steps; i++) b.Step();
                string n = (elapsed / (30 * steps)).ToString("F6", culture);
                lines.Add($"{s};{m};{n};{a.Collisions};{a.Missed};{a.Hit};{a.Passed};{a.FastSteps};" +
                    $"{a.MaxPenetration.ToString("F4", culture)};{a.CcdLimitHits};{a.State() == b.State()}");
            }
        File.WriteAllLines(dialog.FileName, lines, new UTF8Encoding(true));
        MessageBox.Show("Измерения сохранены.", "Готово");
    }
}
