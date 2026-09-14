using System;
using System.Collections.Generic;
using FleetCommand.Test.Templates;
using Havok;
using VRageMath;

namespace FleetCommand.Test.Physics
{
    /// <summary>
    /// Mass, center of mass, and inertia for one rigid grid, combined from the
    /// template's block mass elements by <see cref="HkInertiaTensorComputer"/>
    /// exactly as the game combines them (<c>MyPhysicsBody.UpdateMassProps</c>
    /// at MyPhysicsBody.cs:2867: <c>CombineMassProperties</c> then
    /// <c>RigidBody.SetMassProperties</c>). Each block contributes a box
    /// volume-mass element at its grid-local transform, mirroring
    /// <c>MyCubeBlockCollector.AddMass</c> /
    /// <c>MyPhysicsHelper: ComputeBoxVolumeMassProperties(size/2, mass)</c>.
    /// </summary>
    public sealed class MassProperties
    {
        /// <summary>Total body mass in kilograms (dry + inventory).</summary>
        public float TotalMass { get; }

        /// <summary>Center of mass in grid-local meters (relative to grid origin).</summary>
        public Vector3 CenterOfMass { get; }

        /// <summary>Inertia tensor about the center of mass, grid-local frame (kg·m²).</summary>
        public Matrix InertiaTensor { get; }

        /// <summary>Inverse inertia tensor about the center of mass, grid-local frame.</summary>
        public Matrix InverseInertiaTensor { get; }

        private MassProperties(float totalMass, Vector3 centerOfMass, Matrix inertiaTensor, Matrix inverseInertiaTensor)
        {
            TotalMass = totalMass;
            CenterOfMass = centerOfMass;
            InertiaTensor = inertiaTensor;
            InverseInertiaTensor = inverseInertiaTensor;
        }

        /// <summary>
        /// Combines a ship template's physical blocks (dry + inventory mass)
        /// into one body's mass properties using Havok's
        /// <see cref="HkInertiaTensorComputer.CombineMassProperties"/>, then
        /// also produces the analytic inverse tensor.
        /// </summary>
        public static MassProperties FromShipTemplate(ShipTemplate template)
        {
            if (template == null)
            {
                throw new ArgumentNullException(nameof(template));
            }

            List<HkMassElement> elements = new List<HkMassElement>(template.Blocks.Count * 2);
            foreach (TemplateBlock block in template.Blocks)
            {
                HkMassProperties box = HkInertiaTensorComputer.ComputeBoxVolumeMassProperties(
                    block.HalfExtents, block.DryMass);
                elements.Add(new HkMassElement
                {
                    Properties = box,
                    Tranform = Matrix.CreateTranslation(block.CenterLocal),
                });

                if (block.InventoryMass > 0f)
                {
                    // Inventory mass is a second box element at the block's
                    // volume and position, mirroring
                    // MyGridShape.CollectBlockInventories (MyGridShape.cs:484+).
                    HkMassProperties inventory = HkInertiaTensorComputer.ComputeBoxVolumeMassProperties(
                        block.HalfExtents, block.InventoryMass);
                    elements.Add(new HkMassElement
                    {
                        Properties = inventory,
                        Tranform = Matrix.CreateTranslation(block.CenterLocal),
                    });
                }
            }

            return FromMassElements(elements);
        }

        /// <summary>
        /// Combines the given mass elements (grid-local boxes) with Havok,
        /// mirroring the game's combine path.
        /// </summary>
        public static MassProperties FromMassElements(IEnumerable<HkMassElement> massElements)
        {
            if (massElements == null)
            {
                throw new ArgumentNullException(nameof(massElements));
            }

            HkMassElement[] array = new List<HkMassElement>(massElements).ToArray();
            HkMassProperties combined;
            HkInertiaTensorComputer.CombineMassProperties(
                new Span<HkMassElement>(array, 0, array.Length), out combined);

            return new MassProperties(
                combined.Mass,
                combined.CenterOfMass,
                combined.InertiaTensor,
                Inverse3x3(combined.InertiaTensor));
        }

        /// <summary>
        /// Analytic reference: combines box mass elements (mass + half-extents
        /// + grid-local center) into total mass, center of mass and inertia
        /// about the combined center, using the closed-form box formula and
        /// the parallel-axis theorem — never Havok. Used by conformance
        /// tests to validate the Havok combine result independently.
        /// </summary>
        public static MassProperties AnalyticFromBoxes(IEnumerable<MassElement> boxes)
        {
            double totalMass = 0.0;
            Vector3D com = Vector3D.Zero;
            List<MassElement> list = new List<MassElement>(boxes);
            foreach (MassElement box in list)
            {
                totalMass += box.Mass;
                com += new Vector3D(box.CenterLocal) * box.Mass;
            }
            if (totalMass <= 0.0)
            {
                Matrix identity = Matrix.Identity;
                return new MassProperties(0f, Vector3.Zero, identity, identity);
            }
            com /= totalMass;

            // Sum each box's own inertia (about its own center) plus the
            // parallel-axis transfer from the box center to the combined COM.
            MatrixD sumInertia = MatrixD.Identity;
            ZeroOutRotation(ref sumInertia);
            foreach (MassElement box in list)
            {
                MatrixD own = BoxInertiaAboutCenter(box.Mass, box.HalfExtents);
                Vector3D offset = new Vector3D(box.CenterLocal) - com;
                sumInertia = Add3x3(sumInertia, own, ParallelAxisTransfer(box.Mass, offset));
            }

            Matrix inertia = (Matrix)sumInertia;
            return new MassProperties(
                (float)totalMass,
                (Vector3)com,
                inertia,
                Inverse3x3(inertia));
        }

        /// <summary>
        /// A single rigid-box mass element in grid-local meters.
        /// </summary>
        public struct MassElement
        {
            public float Mass;
            public Vector3 HalfExtents;
            public Vector3 CenterLocal;
        }

        /// <summary>
        /// Solid rectangular-box inertia about its own center (diagonal):
        /// Ix = m/3·(hy²+hz²) about X, and symmetrically — the standard
        /// rigid box formula for full sides (2·h).
        /// </summary>
        public static MatrixD BoxInertiaAboutCenter(double mass, Vector3 halfExtents)
        {
            double hx = halfExtents.X, hy = halfExtents.Y, hz = halfExtents.Z;
            MatrixD result = MatrixD.Identity;
            result.M11 = mass / 3.0 * (hy * hy + hz * hz);
            result.M22 = mass / 3.0 * (hx * hx + hz * hz);
            result.M33 = mass / 3.0 * (hx * hx + hy * hy);
            return result;
        }

        /// <summary>
        /// Parallel-axis transfer: inertia added when a point mass is
        /// displaced from the reference axis by <paramref name="offset"/>:
        /// m · (|r|²·E − r·rᵀ).
        /// </summary>
        public static MatrixD ParallelAxisTransfer(double mass, Vector3D offset)
        {
            double r2 = offset.LengthSquared();
            MatrixD result = MatrixD.Identity;
            result.M11 = mass * (r2 - offset.X * offset.X);
            result.M12 = -mass * offset.X * offset.Y;
            result.M13 = -mass * offset.X * offset.Z;
            result.M21 = -mass * offset.Y * offset.X;
            result.M22 = mass * (r2 - offset.Y * offset.Y);
            result.M23 = -mass * offset.Y * offset.Z;
            result.M31 = -mass * offset.Z * offset.X;
            result.M32 = -mass * offset.Z * offset.Y;
            result.M33 = mass * (r2 - offset.Z * offset.Z);
            return result;
        }

        private static MatrixD Add3x3(MatrixD a, MatrixD b, MatrixD c)
        {
            MatrixD result = MatrixD.Identity;
            result.M11 = a.M11 + b.M11 + c.M11;
            result.M12 = a.M12 + b.M12 + c.M12;
            result.M13 = a.M13 + b.M13 + c.M13;
            result.M21 = a.M21 + b.M21 + c.M21;
            result.M22 = a.M22 + b.M22 + c.M22;
            result.M23 = a.M23 + b.M23 + c.M23;
            result.M31 = a.M31 + b.M31 + c.M31;
            result.M32 = a.M32 + b.M32 + c.M32;
            result.M33 = a.M33 + b.M33 + c.M33;
            return result;
        }

        private static void ZeroOutRotation(ref MatrixD matrix)
        {
            matrix.M11 = 0; matrix.M12 = 0; matrix.M13 = 0;
            matrix.M21 = 0; matrix.M22 = 0; matrix.M23 = 0;
            matrix.M31 = 0; matrix.M32 = 0; matrix.M33 = 0;
        }

        /// <summary>
        /// Inverts the upper-left 3x3 of <paramref name="matrix"/> by
        /// adjugate. The inertia tensor is symmetric; the result is used as
        /// the analytic inverse-inertia reference.
        /// </summary>
        public static Matrix Inverse3x3(Matrix matrix)
        {
            double a = matrix.M11, b = matrix.M12, c = matrix.M13;
            double d = matrix.M21, e = matrix.M22, f = matrix.M23;
            double g = matrix.M31, h = matrix.M32, i = matrix.M33;

            double det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
            if (Math.Abs(det) < 1e-12)
            {
                return Matrix.Identity;
            }
            double inv = 1.0 / det;

            Matrix result = Matrix.Identity;
            result.M11 = (float)((e * i - f * h) * inv);
            result.M12 = (float)((f * g - d * i) * inv);
            result.M13 = (float)((d * h - e * g) * inv);
            result.M21 = (float)((c * h - b * i) * inv);
            result.M22 = (float)((a * i - c * g) * inv);
            result.M23 = (float)((b * g - a * h) * inv);
            result.M31 = (float)((b * f - c * e) * inv);
            result.M32 = (float)((c * d - a * f) * inv);
            result.M33 = (float)((a * e - b * d) * inv);
            return result;
        }
    }
}
