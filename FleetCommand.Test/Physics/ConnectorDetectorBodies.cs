using System;
using FleetCommand.Test.SpaceEngineers;
using FleetCommand.Test.Templates;
using Havok;
using VRageMath;

namespace FleetCommand.Test.Physics
{
    /// <summary>MyShipConnector.LoadDummies/CreatePhysicsBody/CreateBodyConstraint:
    /// two native dynamic phantom bodies fixed to their owning grid, even disabled
    /// or unconnected. Their default 1kg masses are not part of CalculateShipMass.</summary>
    internal sealed class ConnectorDetectorBodies : IDisposable
    {
        private readonly SimCubeGrid _grid;
        private readonly Matrix _blockFrame;
        private readonly Vector3 _center;
        private readonly HkRigidBody[] _bodies = new HkRigidBody[2];
        private readonly HkConstraint[] _constraints = new HkConstraint[2];
        private readonly HkFixedConstraintData[] _data = new HkFixedConstraintData[2];
        private bool _disposed;

        internal ConnectorDetectorBodies(SimCubeGrid grid, TemplateBlock template)
        {
            _grid = grid;
            _blockFrame = template.LocalTransform;
            Matrix dummy = template.ConnectorDetectorDummy ?? Matrix.CreateScale(template.HalfExtents * 2);
            _center = Matrix.Normalize(dummy).Translation;
            Vector3 halfExtents = dummy.Scale / 2;
            try
            {
                using (HkAccessControl.PushState(HkAccessControl.AccessState.Exclusive))
                {
                    for (int i = 0; i < 2; i++)
                    {
                        HkShape volume = i == 0 ? new HkSphereShape(halfExtents.AbsMax()).Base
                            : new HkBoxShape(halfExtents).Base;
                        var phantom = new HkPhantomCallbackShape(EnterDetector, LeaveDetector);
                        var shape = new HkBvShape(volume, phantom, HkReferencePolicy.TakeOwnership);
                        var info = new HkRigidBodyCinfo();
                        try
                        {
                            info.Shape = shape;
                            info.MotionType = HkMotionType.Dynamic;
                            info.QualityType = HkCollidableQualityType.Moving;
                            info.SolverDeactivation = HkSolverDeactivation.Low;
                            info.LinearDamping = 0;
                            info.AngularDamping = 0;
                            // MyPhysicsBody.GetInfoFromFlags(RBF_UNLOCKED_SPEEDS).
                            info.MaxLinearVelocity = grid.World.Profile.MaxLinearVelocity * 10;
                            info.MaxAngularVelocity = grid.World.Profile.MaxAngularVelocityLarge * 10;
                            // Keep native default mass/inertia: the source supplies null massProperties.
                            _bodies[i] = new HkRigidBody(info);
                        }
                        finally { shape.Base.RemoveReference(); info.Dispose(); }
                        _bodies[i].ContactPointCallbackEnabled = true;
                        SetDetectorState(_bodies[i]);
                        uint filter = grid.Body.Body.GetCollisionFilterInfo();
                        _bodies[i].SetCollisionFilterInfo(HkGroupFilter.CalcFilterInfo(grid.Body.Body.Layer,
                            HkGroupFilter.GetSystemGroupFromFilterInfo(filter), 1, 1));
                        grid.World.AddAuxiliaryBody(_bodies[i]);
                        var data = new HkFixedConstraintData();
                        _data[i] = data;
                        data.SetSolvingMethod(HkSolvingMethod.MethodStabilized);
                        data.SetInertiaStabilizationFactor(1);
                        Matrix pivotA = _blockFrame, pivotB = Matrix.CreateTranslation(-_center);
                        data.SetInBodySpaceInternal(ref pivotA, ref pivotB);
                        var constraint = new HkConstraint(grid.Body.Body, _bodies[i], data);
                        _constraints[i] = constraint;
                        constraint.WantRuntime = true;
                        grid.World.AddConstraint(constraint);
                    }
                    grid.Body.StateChanged += SynchronizeState;
                }
            }
            catch { Dispose(); throw; }
        }

        private static void EnterDetector(HkPhantomCallbackShape shape, HkRigidBody body) { }
        private static void LeaveDetector(HkPhantomCallbackShape shape, HkRigidBody body) { }

        private void SetDetectorState(HkRigidBody detector)
        {
            // MyPhysicsBody.GetRigidBodyMatrix uses entity orientation and its Center
            // offset on activation; the raw dummy's orientation is not the final body pose.
            Matrix local = Matrix.CreateTranslation(_center) * _blockFrame;
            Matrix pose = local * _grid.Body.Body.GetRigidBodyMatrix();
            detector.SetWorldMatrix(pose);
            detector.LinearVelocity = _grid.Body.Body.GetVelocityAtPoint(pose.Translation);
            detector.AngularVelocity = _grid.Body.Body.AngularVelocity;
        }

        private void SynchronizeState()
        {
            if (_disposed) return;
            using (HkAccessControl.PushState(HkAccessControl.AccessState.Exclusive))
                foreach (var body in _bodies) if (body != null) SetDetectorState(body);
        }

        public void Dispose()
        {
            if (_disposed) return;
            _grid.Body.StateChanged -= SynchronizeState;
            using (HkAccessControl.PushState(HkAccessControl.AccessState.Exclusive))
            {
                for (int i = 1; i >= 0; i--)
                {
                    if (_constraints[i] != null) { _grid.World.RemoveConstraint(_constraints[i]); _constraints[i].Dispose(); }
                    if (_bodies[i] != null) { _grid.World.RemoveAuxiliaryBody(_bodies[i]); _bodies[i].Dispose(); }
                    if (_data[i] != null) _data[i].Dispose();
                }
            }
            _disposed = true;
        }
    }
}
