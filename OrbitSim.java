import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Point;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.util.ArrayList;
import java.util.List;
import javax.swing.Timer;
import javax.swing.JButton;
import javax.swing.JComponent;
import javax.swing.JFrame;

class OrbitSim extends JFrame {
    private static class Vector2d {
        public static Vector2d ZERO = new Vector2d(0, 0);

        private double x;
        private double y;

        public Vector2d(double x, double y) {
            this.x = x;
            this.y = y;
        }

        public double getX() {
            return x;
        }

        public double getY() {
            return y;
        }

        public Vector2d multiplyBy(double factor) {
            return new Vector2d(x*factor, y*factor);
        }

        public Vector2d devidedBy(double factor) {
            return new Vector2d(x/factor, y/factor);
        }

        public Vector2d add(Vector2d vector2d) {
            return new Vector2d(x + vector2d.x, y + vector2d.y);
        }

        public Vector2d subtractBy(Vector2d vector2d) {
            return new Vector2d(x - vector2d.x, y - vector2d.y);
        }

        public double getLength() {
            return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
        }
    }

    private static class PointMass {
        private Vector2d position;
        private Vector2d velocity;
        private double mass;

        public PointMass(Vector2d position, Vector2d velocity, double mass) {
            this.position = position;
            this.velocity = velocity;
            this.mass = mass;
        }

        public void setPosition(Vector2d position) {
            this.position = position;
        }
        
        public Vector2d getPosition() {
            return position;
        }

        public void setVelocity(Vector2d velocity) {
            this.velocity = velocity;
        }

        public Vector2d getVelocity() {
            return velocity;
        }

        public double getMass() {
            return mass;
        }
    }

    private class Simulator implements ActionListener {
        @Override
        public void actionPerformed(ActionEvent arg0) {
            synchronized (bodies) {
                // Save copy of bodies
                List<PointMass> copied = new ArrayList<>(bodies);

                for (PointMass pointMass : copied) {
                    // We want to know combined force applied to this point mass
                    Vector2d pos = pointMass.getPosition();
                    Vector2d velDelta = Vector2d.ZERO;

                    for (PointMass affect : copied) {
                        if (pointMass == affect)
                            continue;

                        // Calculate force of this object "affect" applies to this object
                        // Acceleration of Object 1 = G * M1 / ||^p2 - ^p1||^3 * (^p2 - ^p1)
                        Vector2d affPos = affect.getPosition();
                        double affMass = affect.getMass();
                        Vector2d distance = affPos.subtractBy(pos);
                        
                        // ^p2 - ^p1
                        Vector2d localAccel = affPos.subtractBy(pos);
                        // G*M1
                        localAccel = localAccel.multiplyBy(GRAVITY_CONSTANT*affMass);
                        // ||^p2 - ^p1||^3
                        localAccel = localAccel.devidedBy(Math.pow(distance.getLength(), 3));
                        // Multiply by tick speed
                        localAccel = localAccel.multiplyBy(TICK_SPEED).devidedBy(1000);

                        // Add it to delta
                        velDelta = velDelta.add(localAccel);
                    }

                    // Replace velocity and position
                    Vector2d newVel = pointMass.getVelocity().add(velDelta);
                    Vector2d newPos = pointMass.getPosition().add(newVel);
                    pointMass.setVelocity(newVel);
                    pointMass.setPosition(newPos);
                }
            }

            repaint();
        }
    }

    private class OrbitSimComponent extends JComponent {
        public OrbitSimComponent() {
            // Add listener
            addMouseListener(new MouseAdapter() {
                private Point enterPoint;
                private long enterTime;

                @Override
                public void mousePressed(MouseEvent e) {
                    enterPoint = e.getPoint();
                    enterTime = e.getWhen();
                }

                @Override
                public void mouseReleased(MouseEvent e) {
                    Vector2d frameSize = new Vector2d(getWidth(), getHeight());

                    synchronized (bodies) {
                        PointMass focus;

                        if (bodies.size() == 0) {
                            // Dummy
                            focus = new PointMass(Vector2d.ZERO, Vector2d.ZERO, 0);
                        } else {
                            focus = bodies.get(0);
                        }

                        double mass = Math.pow(10, (e.getWhen() - enterTime) / 200D);

                        Point endPoint = e.getPoint();
                        Vector2d pos = new Vector2d(endPoint.getX(), endPoint.getY())
                            .subtractBy(frameSize.devidedBy(2))
                            .multiplyBy(Math.pow(10, scaleFactor))
                            .add(focus.getPosition());
                        Vector2d vel = new Vector2d(endPoint.getX() - enterPoint.getX(), endPoint.getY() - enterPoint.getY())
                            .multiplyBy(Math.pow(10, scaleFactor - 3))
                            .add(focus.getVelocity());

                        bodies.add(new PointMass(pos, vel, mass));
                    }

                    // Do repaint
                    repaint();
                }
            });
        }

        @Override
        public void paintComponent(Graphics g) {
            g.setColor(Color.BLACK);
            Vector2d frameSize = new Vector2d(getWidth(), getHeight());
    
            synchronized (bodies) {
                if (bodies.size() < 1) return;
                Vector2d focusPos = bodies.get(0).getPosition();

                for (PointMass pointMass : bodies) {
                    Vector2d position = pointMass.getPosition();
                    Vector2d relativePos = position.subtractBy(focusPos)
                        .multiplyBy(Math.pow(10, -scaleFactor))
                        .add(frameSize.devidedBy(2));
                    double mass = pointMass.getMass();
					double diameter = Math.log10(mass) * 10;
                    
                    g.fillOval((int) Math.round(relativePos.getX() - diameter/2),
                            (int) Math.round(relativePos.getY() - diameter/2),
                            (int) diameter,
                            (int) diameter
                    );
                    g.drawString("Mass:" + String.valueOf(Math.round(mass)), (int) (relativePos.getX() + diameter), (int) (relativePos.getY() + diameter));
                }
            }
        }
    }

    private static int TICK_SPEED = 1;
    private static double GRAVITY_CONSTANT = 60000;
    private List<PointMass> bodies = new ArrayList<>();
    private Timer timer;
    private double scaleFactor = 2;

    public OrbitSim() {
        super("Orbit Simulator");
        setSize(500, 500);
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        setLayout(new BorderLayout());

        add(new OrbitSimComponent(), BorderLayout.CENTER);


        // Setting up timer
        timer = new Timer(TICK_SPEED, new Simulator());
        this.timer.start();
    }

    public static void main(String[] args) {
        new OrbitSim().setVisible(true);
    }

}