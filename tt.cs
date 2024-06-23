namespace n_body_problem {
    public class Simulator {

        private const double GravitationalConstant = 6.67430e-11;
        public List<Body> Bodies { get; } = new List<Body>();
        public void AddBody(Body body) {
            Bodies.Add(body);
        }

        public void UpdateTime(double timeStep) {
            foreach (var obj in Bodies) {
                var netForce = new Vector(0, 0);

                foreach (var otherObj in Bodies) {
                    if (obj == otherObj) continue;
                    var direction = obj.Position - otherObj.Position;
                    var distance = direction.Magnitude;
                    netForce += direction.Normalize() * GravitationalConstant * obj.Mass * otherObj.Mass / (distance * distance);
                }

                var acceleration = netForce / obj.Mass;
                obj.Velocity += acceleration * timeStep;
            }

            foreach (var obj in Bodies) {
                obj.Position += obj.Velocity * timeStep;
            }
        }
    }
}

namespace n_body_problem {
    public class Simulator {

        private const double GravitationalConstant = 6.67430e-11;
        public List<Body> Bodies { get; } = new List<Body>();
        public void AddBody(Body body) {
            Bodies.Add(body);
        }

        public void UpdateTime(double timeStep) {
            foreach (var obj in Bodies) {
                var netForce = new Vector(0, 0);

                foreach (var otherObj in Bodies) {
                    if (obj == otherObj) continue;
                    var direction = obj.Position - otherObj.Position;
                    var distance = direction.Magnitude;
                    netForce += direction.Normalize() * GravitationalConstant * obj.Mass * otherObj.Mass / (distance * distance);
                }

                var acceleration = netForce / obj.Mass;
                obj.Velocity += acceleration * timeStep;
            }

            foreach (var obj in Bodies) {
                obj.Position += obj.Velocity * timeStep;
            }
        }
    }
}

namespace n_body_problem {
    public class Vector {
        public Vector(double x, double y) {
            X = x; Y = y;
        }
        public double X { get; set; }
        public double Y { get; set; }
        public double Magnitude => Math.Sqrt(X * X + Y * Y);
        public Vector Normalize() => new Vector(X / Magnitude, Y / Magnitude);
        public static double DotProduct(Vector left, Vector right) => left.X * right.X + left.Y * right.Y;
        public static Vector operator *(Vector vector, double scalar) => new Vector(vector.X * scalar, vector.Y * scalar);
        public static Vector operator /(Vector vector, double scalar) => new Vector(vector.X / scalar, vector.Y / scalar);
        public static Vector operator +(Vector left, Vector right) => new Vector(left.X + right.X, left.Y + right.Y);
        public static Vector operator -(Vector left, Vector right) => new Vector(left.X - right.X, left.Y - right.Y);
    }
}
