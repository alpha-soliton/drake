# -*- coding: utf-8 -*-

import pydrake.systems.framework as mut

import copy
import warnings

import unittest
import numpy as np

from pydrake.autodiffutils import AutoDiffXd
from pydrake.examples.pendulum import PendulumPlant
from pydrake.examples.rimless_wheel import RimlessWheel
from pydrake.symbolic import Expression
from pydrake.systems.analysis import (
    IntegratorBase, IntegratorBase_,
    RungeKutta2Integrator, RungeKutta3Integrator,
    Simulator, Simulator_,
    )
from pydrake.systems.framework import (
    AbstractValue,
    BasicVector, BasicVector_,
    Context, Context_,
    ContinuousState, ContinuousState_,
    Diagram, Diagram_,
    DiagramBuilder, DiagramBuilder_,
    DiscreteUpdateEvent, DiscreteUpdateEvent_,
    DiscreteValues, DiscreteValues_,
    Event, Event_,
    InputPort, InputPort_,
    kUseDefaultName,
    LeafContext, LeafContext_,
    LeafSystem, LeafSystem_,
    OutputPort, OutputPort_,
    Parameters, Parameters_,
    PeriodicEventData,
    PublishEvent, PublishEvent_,
    State, State_,
    Subvector, Subvector_,
    Supervector, Supervector_,
    System, System_,
    SystemOutput, SystemOutput_,
    VectorBase, VectorBase_,
    TriggerType,
    VectorSystem, VectorSystem_,
    )
from pydrake.systems.primitives import (
    Adder, Adder_,
    AffineSystem,
    ConstantValueSource,
    ConstantVectorSource, ConstantVectorSource_,
    Integrator,
    LinearSystem,
    PassThrough,
    SignalLogger,
    ZeroOrderHold,
    )

# TODO(eric.cousineau): The scope of this test file and and `custom_test.py`
# is poor. Move these tests into `framework_test` and `analysis_test`, and
# ensure that the tests reflect this, even if there is some coupling.


class TestGeneral(unittest.TestCase):
    def _check_instantiations(
            self, template, default_cls, supports_symbolic=True):
        self.assertIs(template[None], default_cls)
        self.assertIs(template[float], default_cls)
        self.assertIsNot(template[AutoDiffXd], default_cls)
        if supports_symbolic:
            self.assertIsNot(template[Expression], default_cls)

    def _compare_system_instances(self, lhs, rhs):
        # Compares two different scalar type instantiation instances of a
        # system.
        self.assertEqual(lhs.num_input_ports(), rhs.num_input_ports())
        self.assertEqual(
            lhs.num_output_ports(), rhs.num_output_ports())
        for i in range(lhs.num_input_ports()):
            lhs_port = lhs.get_input_port(i)
            rhs_port = rhs.get_input_port(i)
            self.assertEqual(lhs_port.size(), rhs_port.size())
        for i in range(lhs.num_output_ports()):
            lhs_port = lhs.get_output_port(i)
            rhs_port = rhs.get_output_port(i)
            self.assertEqual(lhs_port.size(), rhs_port.size())

    def test_system_base_api(self):
        # Test a system with a different number of inputs from outputs.
        system = Adder(3, 10)
        self.assertEqual(system.num_input_ports(), 3)
        self.assertEqual(system.num_output_ports(), 1)
        u1 = system.GetInputPort("u1")
        self.assertEqual(u1.get_name(), "u1")
        self.assertIn("u1", u1.GetFullDescription())
        self.assertEqual(u1.get_index(), 1)
        self.assertEqual(u1.size(), 10)
        self.assertIsNotNone(u1.ticket())
        self.assertEqual(system.GetOutputPort("sum").get_index(), 0)
        # TODO(eric.cousineau): Consolidate the main API tests for `System`
        # to this test point.

    def test_context_api(self):
        system = Adder(3, 10)
        context = system.AllocateContext()
        self.assertIsInstance(
            context.get_continuous_state(), ContinuousState)
        self.assertIsInstance(
            context.get_mutable_continuous_state(), ContinuousState)
        self.assertIsInstance(
            context.get_continuous_state_vector(), VectorBase)
        self.assertIsInstance(
            context.get_mutable_continuous_state_vector(), VectorBase)

        context = system.CreateDefaultContext()
        self.assertIsInstance(
            context.get_continuous_state(), ContinuousState)
        self.assertIsInstance(
            context.get_mutable_continuous_state(), ContinuousState)
        self.assertIsInstance(
            context.get_continuous_state_vector(), VectorBase)
        self.assertIsInstance(
            context.get_mutable_continuous_state_vector(), VectorBase)
        self.assertTrue(context.is_stateless())
        self.assertFalse(context.has_only_continuous_state())
        self.assertFalse(context.has_only_discrete_state())
        self.assertEqual(context.num_total_states(), 0)
        # TODO(eric.cousineau): Consolidate main API tests for `Context` here.

        # Test methods with two scalar types.
        for T in [float, AutoDiffXd, Expression]:
            systemT = Adder_[T](3, 10)
            contextT = systemT.CreateDefaultContext()
            for U in [float, AutoDiffXd, Expression]:
                systemU = Adder_[U](3, 10)
                contextU = systemU.CreateDefaultContext()
                contextU.SetTime(0.5)
                contextT.SetTimeStateAndParametersFrom(contextU)
                if T == float:
                    self.assertEqual(contextT.get_time(), 0.5)
                elif T == AutoDiffXd:
                    self.assertEqual(contextT.get_time().value(), 0.5)
                else:
                    self.assertEqual(contextT.get_time().Evaluate(), 0.5)

        pendulum = PendulumPlant()
        context = pendulum.CreateDefaultContext()
        self.assertEqual(context.num_numeric_parameter_groups(), 1)
        self.assertEqual(pendulum.num_numeric_parameter_groups(), 1)
        self.assertTrue(
            context.get_parameters().get_numeric_parameter(0) is
            context.get_numeric_parameter(index=0))
        self.assertTrue(
            context.get_mutable_parameters().get_mutable_numeric_parameter(
                0) is context.get_mutable_numeric_parameter(index=0))
        self.assertEqual(context.num_abstract_parameters(), 0)
        self.assertEqual(pendulum.num_numeric_parameter_groups(), 1)
        # TODO(russt): Bind _Declare*Parameter or find an example with an
        # abstract parameter to actually call this method.
        self.assertTrue(hasattr(context, "get_abstract_parameter"))
        self.assertTrue(hasattr(context, "get_mutable_abstract_parameter"))
        x = np.array([0.1, 0.2])
        context.SetContinuousState(x)
        np.testing.assert_equal(
            context.get_continuous_state_vector().CopyToVector(), x)
        context.SetTimeAndContinuousState(0.3, 2*x)
        np.testing.assert_equal(context.get_time(), 0.3)
        np.testing.assert_equal(
            context.get_continuous_state_vector().CopyToVector(), 2*x)

        # RimlessWheel has a single discrete variable and a bool abstract
        # variable.
        rimless = RimlessWheel()
        context = rimless.CreateDefaultContext()
        x = np.array([1.125])
        context.SetDiscreteState(xd=2 * x)
        np.testing.assert_equal(
            context.get_discrete_state_vector().CopyToVector(), 2 * x)
        context.SetDiscreteState(group_index=0, xd=3 * x)
        np.testing.assert_equal(
            context.get_discrete_state_vector().CopyToVector(), 3 * x)

        def check_abstract_value_zero(context, expected_value):
            # Check through Context, State, and AbstractValues APIs.
            self.assertEqual(context.get_abstract_state(index=0).get_value(),
                             expected_value)
            self.assertEqual(context.get_abstract_state().get_value(
                index=0).get_value(), expected_value)
            self.assertEqual(context.get_state().get_abstract_state()
                             .get_value(index=0).get_value(), expected_value)

        context.SetAbstractState(index=0, value=True)
        check_abstract_value_zero(context, True)
        context.SetAbstractState(index=0, value=False)
        check_abstract_value_zero(context, False)
        value = context.get_mutable_state().get_mutable_abstract_state()\
            .get_mutable_value(index=0)
        value.set_value(True)
        check_abstract_value_zero(context, True)

    def test_event_api(self):
        # TriggerType - existence check.
        TriggerType.kUnknown
        TriggerType.kInitialization
        TriggerType.kForced
        TriggerType.kTimed
        TriggerType.kPeriodic
        TriggerType.kPerStep
        TriggerType.kWitness

        # PublishEvent.
        # TODO(eric.cousineau): Test other event types when it is useful to
        # expose them.

        def callback(context, event): pass

        event = PublishEvent(
            trigger_type=TriggerType.kInitialization, callback=callback)
        self.assertIsInstance(event, Event)
        self.assertEqual(event.get_trigger_type(), TriggerType.kInitialization)

        # Simple discrete-time system.
        system1 = LinearSystem(A=[1], B=[1], C=[1], D=[1], time_period=0.1)
        periodic_data = system1.GetUniquePeriodicDiscreteUpdateAttribute()
        self.assertIsInstance(periodic_data, PeriodicEventData)
        self.assertIsInstance(periodic_data.Clone(), PeriodicEventData)
        periodic_data.period_sec()
        periodic_data.offset_sec()

        # Simple continuous-time system.
        system2 = LinearSystem(A=[1], B=[1], C=[1], D=[1], time_period=0.0)
        periodic_data = system2.GetUniquePeriodicDiscreteUpdateAttribute()
        self.assertIsNone(periodic_data)

    def test_instantiations(self):
        # Quick check of instantiations for given types.
        # N.B. These checks are ordered according to their binding definitions
        # in the corresponding source file.
        # `analysis_py.cc`
        self._check_instantiations(IntegratorBase_, IntegratorBase, False)
        self._check_instantiations(Simulator_, Simulator, False)
        # `framework_py_semantics.cc`
        self._check_instantiations(Context_, Context)
        self._check_instantiations(LeafContext_, LeafContext)
        self._check_instantiations(Event_, Event)
        self._check_instantiations(PublishEvent_, PublishEvent)
        self._check_instantiations(DiscreteUpdateEvent_, DiscreteUpdateEvent)
        self._check_instantiations(DiagramBuilder_, DiagramBuilder)
        self._check_instantiations(OutputPort_, OutputPort)
        self._check_instantiations(SystemOutput_, SystemOutput)
        self._check_instantiations(InputPort_, InputPort)
        self._check_instantiations(Parameters_, Parameters)
        self._check_instantiations(State_, State)
        self._check_instantiations(ContinuousState_, ContinuousState)
        self._check_instantiations(DiscreteValues_, DiscreteValues)
        # `framework_py_systems.cc`
        self._check_instantiations(System_, System)
        self._check_instantiations(LeafSystem_, LeafSystem)
        self._check_instantiations(Diagram_, Diagram)
        self._check_instantiations(VectorSystem_, VectorSystem)
        # `framework_py_values.cc`
        self._check_instantiations(VectorBase_, VectorBase)
        self._check_instantiations(BasicVector_, BasicVector)
        self._check_instantiations(Supervector_, Supervector)
        self._check_instantiations(Subvector_, Subvector)

    def test_scalar_type_conversion(self):
        float_system = Adder(1, 1)
        float_context = float_system.CreateDefaultContext()
        float_system.get_input_port(0).FixValue(float_context, 1.)
        for T in [float, AutoDiffXd, Expression]:
            system = Adder_[T](1, 1)
            # N.B. Current scalar conversion does not permit conversion to and
            # from the same type.
            if T != AutoDiffXd:
                methods = [Adder_[T].ToAutoDiffXd, Adder_[T].ToAutoDiffXdMaybe]
                for method in methods:
                    system_ad = method(system)
                    self.assertIsInstance(system_ad, System_[AutoDiffXd])
                    self._compare_system_instances(system, system_ad)
            if T != Expression:
                methods = [Adder_[T].ToSymbolic, Adder_[T].ToSymbolicMaybe]
                for method in methods:
                    system_sym = method(system)
                    self.assertIsInstance(system_sym, System_[Expression])
                    self._compare_system_instances(system, system_sym)
            context = system.CreateDefaultContext()
            system.FixInputPortsFrom(other_system=float_system,
                                     other_context=float_context,
                                     target_context=context)
            u = system.get_input_port(0).Eval(context)
            self.assertEqual(len(u), 1)
            if T == float:
                self.assertEqual(u[0], 1.)
            elif T == AutoDiffXd:
                self.assertEqual(u[0].value(), 1.)
            else:
                self.assertEqual(u[0].Evaluate(), 1.)

    def test_simulator_ctor(self):
        # Tests a simple simulation for supported scalar types.
        for T in [float, AutoDiffXd]:
            # Create simple system.
            system = ConstantVectorSource_[T]([1.])

            def check_output(context):
                # Check number of output ports and value for a given context.
                output = system.AllocateOutput()
                self.assertEqual(output.num_ports(), 1)
                system.CalcOutput(context=context, outputs=output)
                if T == float:
                    value = output.get_vector_data(0).get_value()
                    self.assertTrue(np.allclose([1], value))
                elif T == AutoDiffXd:
                    value = output.get_vector_data(0)._get_value_copy()
                    # TODO(eric.cousineau): Define `isfinite` ufunc, if
                    # possible, to use for `np.allclose`.
                    self.assertEqual(value.shape, (1,))
                    self.assertEqual(value[0], AutoDiffXd(1.))
                else:
                    raise RuntimeError("Bad T: {}".format(T))

            # Create simulator with basic constructor.
            simulator = Simulator_[T](system)
            simulator.Initialize()
            simulator.set_target_realtime_rate(0)
            simulator.set_publish_every_time_step(True)
            self.assertTrue(simulator.get_context() is
                            simulator.get_mutable_context())
            check_output(simulator.get_context())
            simulator.AdvanceTo(1)

            # Create simulator specifying context.
            context = system.CreateDefaultContext()
            context.SetTime(0.)

            context.SetAccuracy(1e-4)
            self.assertEqual(context.get_accuracy(), 1e-4)

            # @note `simulator` now owns `context`.
            simulator = Simulator_[T](system, context)
            self.assertTrue(simulator.get_context() is context)
            check_output(context)
            simulator.AdvanceTo(1)
            simulator.AdvancePendingEvents()

    def test_copy(self):
        # Copy a context using `deepcopy` or `clone`.
        system = ConstantVectorSource([1])
        context = system.CreateDefaultContext()
        context_copies = [
            copy.copy(context),
            copy.deepcopy(context),
            context.Clone(),
        ]
        # TODO(eric.cousineau): Compare copies.
        for context_copy in context_copies:
            self.assertTrue(context_copy is not context)

    def test_diagram_simulation(self):
        # Similar to: //systems/framework:diagram_test, ExampleDiagram
        size = 3

        builder = DiagramBuilder()
        adder0 = builder.AddSystem(Adder(2, size))
        adder0.set_name("adder0")

        adder1 = builder.AddSystem(Adder(2, size))
        adder1.set_name("adder1")

        integrator = builder.AddSystem(Integrator(size))
        integrator.set_name("integrator")

        builder.Connect(adder0.get_output_port(0), adder1.get_input_port(0))
        builder.Connect(adder1.get_output_port(0),
                        integrator.get_input_port(0))

        # Exercise naming variants.
        builder.ExportInput(adder0.get_input_port(0))
        builder.ExportInput(adder0.get_input_port(1), kUseDefaultName)
        builder.ExportInput(adder1.get_input_port(1), "third_input")
        builder.ExportOutput(integrator.get_output_port(0), "result")

        diagram = builder.Build()
        self.assertEqual(adder0.get_name(), "adder0")
        self.assertEqual(diagram.GetSubsystemByName("adder0"), adder0)
        # TODO(eric.cousineau): Figure out unicode handling if needed.
        # See //systems/framework/test/diagram_test.cc:349 (sha: bc84e73)
        # for an example name.
        diagram.set_name("test_diagram")

        simulator = Simulator(diagram)
        context = simulator.get_mutable_context()

        # Create and attach inputs.
        # TODO(eric.cousineau): Not seeing any assertions being printed if no
        # inputs are connected. Need to check this behavior.
        input0 = np.array([0.1, 0.2, 0.3])
        diagram.get_input_port(0).FixValue(context, input0)
        input1 = np.array([0.02, 0.03, 0.04])
        diagram.get_input_port(1).FixValue(context, input1)
        # Test the BasicVector overload.
        input2 = BasicVector([0.003, 0.004, 0.005])
        diagram.get_input_port(2).FixValue(context, input2)

        # Test __str__ methods.
        self.assertRegexpMatches(str(context), "integrator")
        self.assertEqual(str(input2), "[0.003, 0.004, 0.005]")

        # Initialize integrator states.
        integrator_xc = (
            diagram.GetMutableSubsystemState(integrator, context)
                   .get_mutable_continuous_state().get_vector())
        integrator_xc.SetFromVector([0, 1, 2])

        simulator.Initialize()

        # Simulate briefly, and take full-context snapshots at intermediate
        # points.
        n = 6
        times = np.linspace(0, 1, n)
        context_log = []
        for t in times:
            simulator.AdvanceTo(t)
            # Record snapshot of *entire* context.
            context_log.append(context.Clone())

        xc_initial = np.array([0, 1, 2])
        xc_final = np.array([0.123, 1.234, 2.345])

        for i, context_i in enumerate(context_log):
            t = times[i]
            self.assertEqual(context_i.get_time(), t)
            xc = context_i.get_continuous_state_vector().CopyToVector()
            xc_expected = (float(i) / (n - 1) * (xc_final - xc_initial) +
                           xc_initial)
            self.assertTrue(np.allclose(xc, xc_expected))

    def test_simulator_integrator_manipulation(self):
        system = ConstantVectorSource([1])

        # Create simulator with basic constructor.
        simulator = Simulator(system)
        simulator.Initialize()
        simulator.set_target_realtime_rate(0)

        integrator = simulator.get_mutable_integrator()

        target_accuracy = 1E-6
        integrator.set_target_accuracy(target_accuracy)
        self.assertEqual(integrator.get_target_accuracy(), target_accuracy)

        maximum_step_size = 0.2
        integrator.set_maximum_step_size(maximum_step_size)
        self.assertEqual(integrator.get_maximum_step_size(), maximum_step_size)

        minimum_step_size = 2E-2
        integrator.set_requested_minimum_step_size(minimum_step_size)
        self.assertEqual(integrator.get_requested_minimum_step_size(),
                         minimum_step_size)

        integrator.set_throw_on_minimum_step_size_violation(True)
        self.assertTrue(integrator.get_throw_on_minimum_step_size_violation())

        integrator.set_fixed_step_mode(True)
        self.assertTrue(integrator.get_fixed_step_mode())

        const_integrator = simulator.get_integrator()
        self.assertTrue(const_integrator is integrator)

        # Test context-less constructors for
        # integrator types.
        test_integrator = RungeKutta2Integrator(
            system=system, max_step_size=0.01)
        test_integrator = RungeKutta3Integrator(system=system)

        # Test simulator's reset_integrator,
        # and also the full constructors for
        # all integrator types.
        simulator.reset_integrator(
            RungeKutta2Integrator(
                system=system,
                max_step_size=0.01,
                context=simulator.get_mutable_context()))

        simulator.reset_integrator(
            RungeKutta3Integrator(
                system=system,
                context=simulator.get_mutable_context()))

    def test_abstract_output_port_eval(self):
        model_value = AbstractValue.Make("Hello World")
        source = ConstantValueSource(copy.copy(model_value))
        context = source.CreateDefaultContext()
        output_port = source.get_output_port(0)

        value = output_port.Eval(context)
        self.assertEqual(type(value), type(model_value.get_value()))
        self.assertEqual(value, model_value.get_value())

        value_abs = output_port.EvalAbstract(context)
        self.assertEqual(type(value_abs), type(model_value))
        self.assertEqual(value_abs.get_value(), model_value.get_value())

    def test_vector_output_port_eval(self):
        np_value = np.array([1., 2., 3.])
        model_value = AbstractValue.Make(BasicVector(np_value))
        source = ConstantVectorSource(np_value)
        context = source.CreateDefaultContext()
        output_port = source.get_output_port(0)

        value = output_port.Eval(context)
        self.assertEqual(type(value), np.ndarray)
        np.testing.assert_equal(value, np_value)

        value_abs = output_port.EvalAbstract(context)
        self.assertEqual(type(value_abs), type(model_value))
        self.assertEqual(type(value_abs.get_value().get_value()), np.ndarray)
        np.testing.assert_equal(value_abs.get_value().get_value(), np_value)

        basic = output_port.EvalBasicVector(context)
        self.assertEqual(type(basic), BasicVector)
        self.assertEqual(type(basic.get_value()), np.ndarray)
        np.testing.assert_equal(basic.get_value(), np_value)

    def test_abstract_input_port_eval(self):
        model_value = AbstractValue.Make("Hello World")
        system = PassThrough(copy.copy(model_value))
        context = system.CreateDefaultContext()
        fixed = system.get_input_port(0).FixValue(context,
                                                  copy.copy(model_value))
        self.assertIsInstance(fixed.GetMutableData(), AbstractValue)
        input_port = system.get_input_port(0)

        value = input_port.Eval(context)
        self.assertEqual(type(value), type(model_value.get_value()))
        self.assertEqual(value, model_value.get_value())

        value_abs = input_port.EvalAbstract(context)
        self.assertEqual(type(value_abs), type(model_value))
        self.assertEqual(value_abs.get_value(), model_value.get_value())

    def test_vector_input_port_eval(self):
        np_value = np.array([1., 2., 3.])
        model_value = AbstractValue.Make(BasicVector(np_value))
        system = PassThrough(len(np_value))
        context = system.CreateDefaultContext()
        system.get_input_port(0).FixValue(context, np_value)
        input_port = system.get_input_port(0)

        value = input_port.Eval(context)
        self.assertEqual(type(value), np.ndarray)
        np.testing.assert_equal(value, np_value)

        value_abs = input_port.EvalAbstract(context)
        self.assertEqual(type(value_abs), type(model_value))
        self.assertEqual(type(value_abs.get_value().get_value()), np.ndarray)
        np.testing.assert_equal(value_abs.get_value().get_value(), np_value)

        basic = input_port.EvalBasicVector(context)
        self.assertEqual(type(basic), BasicVector)
        self.assertEqual(type(basic.get_value()), np.ndarray)
        np.testing.assert_equal(basic.get_value(), np_value)

    def test_abstract_input_port_fix_string(self):
        model_value = AbstractValue.Make("")
        system = PassThrough(copy.copy(model_value))
        context = system.CreateDefaultContext()
        input_port = system.get_input_port(0)

        # Fix to a literal.
        input_port.FixValue(context, "Alpha")
        value = input_port.Eval(context)
        self.assertEqual(type(value), type(model_value.get_value()))
        self.assertEqual(value, "Alpha")

        # Fix to a type-erased string.
        input_port.FixValue(context, AbstractValue.Make("Bravo"))
        value = input_port.Eval(context)
        self.assertEqual(type(value), type(model_value.get_value()))
        self.assertEqual(value, "Bravo")

        # Fix to a non-string.
        with self.assertRaises(RuntimeError):
            # A RuntimeError occurs when the Context detects that the
            # type-erased Value objects are incompatible.
            input_port.FixValue(context, AbstractValue.Make(1))
        with self.assertRaises(TypeError):
            # A TypeError occurs when pybind Value.set_value cannot match any
            # overload for how to assign the argument into the erased storage.
            input_port.FixValue(context, 1)
        with self.assertRaises(TypeError):
            input_port.FixValue(context, np.array([2.]))

    def test_abstract_input_port_fix_object(self):
        # The port type is py::object, not any specific C++ type.
        model_value = AbstractValue.Make(object())
        system = PassThrough(copy.copy(model_value))
        context = system.CreateDefaultContext()
        input_port = system.get_input_port(0)

        # Fix to a type-erased py::object.
        input_port.FixValue(context, AbstractValue.Make(object()))

        # Fix to an int.
        input_port.FixValue(context, 1)
        value = input_port.Eval(context)
        self.assertEqual(type(value), int)
        self.assertEqual(value, 1)

        # Fixing to an explicitly-typed Value instantation is an error ...
        with self.assertRaises(RuntimeError):
            input_port.FixValue(context, AbstractValue.Make("string"))
        # ... but implicit typing works just fine.
        input_port.FixValue(context, "string")
        value = input_port.Eval(context)
        self.assertEqual(type(value), str)
        self.assertEqual(value, "string")

    def test_vector_input_port_fix(self):
        np_zeros = np.array([0.])
        model_value = AbstractValue.Make(BasicVector(np_zeros))
        system = PassThrough(len(np_zeros))
        context = system.CreateDefaultContext()
        input_port = system.get_input_port(0)

        # Fix to a scalar.
        input_port.FixValue(context, 1.)
        value = input_port.Eval(context)
        self.assertEqual(type(value), np.ndarray)
        np.testing.assert_equal(value, np.array([1.]))

        # Fix to an ndarray.
        input_port.FixValue(context, np.array([2.]))
        value = input_port.Eval(context)
        self.assertEqual(type(value), np.ndarray)
        np.testing.assert_equal(value, np.array([2.]))

        # Fix to a BasicVector.
        input_port.FixValue(context, BasicVector([3.]))
        value = input_port.Eval(context)
        self.assertEqual(type(value), np.ndarray)
        np.testing.assert_equal(value, np.array([3.]))

        # Fix to a type-erased BasicVector.
        input_port.FixValue(context, AbstractValue.Make(BasicVector([4.])))
        value = input_port.Eval(context)
        self.assertEqual(type(value), np.ndarray)
        np.testing.assert_equal(value, np.array([4.]))

        # Fix to wrong-sized vector.
        with self.assertRaises(RuntimeError):
            input_port.FixValue(context, np.array([0., 1.]))
        with self.assertRaises(RuntimeError):
            input_port.FixValue(
                context, AbstractValue.Make(BasicVector([0., 1.])))

        # Fix to a non-vector.
        with self.assertRaises(TypeError):
            # A TypeError occurs when pybind Value.set_value cannot match any
            # overload for how to assign the argument into the erased storage.
            input_port.FixValue(context, "string")
        with self.assertRaises(RuntimeError):
            # A RuntimeError occurs when the Context detects that the
            # type-erased Value objects are incompatible.
            input_port.FixValue(context, AbstractValue.Make("string"))

    def test_context_fix_input_port(self):
        # WARNING: This is not the recommend workflow; instead, use
        # `InputPort.FixValue` instead. This is here just for testing /
        # coverage purposes.
        dt = 0.1  # Arbitrary.
        system_vec = ZeroOrderHold(period_sec=dt, vector_size=1)
        context_vec = system_vec.CreateDefaultContext()
        context_vec.FixInputPort(index=0, data=[0.])
        context_vec.FixInputPort(index=0, vec=BasicVector([0.]))
        # Test abstract.
        model_value = AbstractValue.Make("Hello")
        system_abstract = ZeroOrderHold(
            period_sec=dt, abstract_model_value=model_value.Clone())
        context_abstract = system_abstract.CreateDefaultContext()
        context_abstract.FixInputPort(index=0, value=model_value.Clone())
