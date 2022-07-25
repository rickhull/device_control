[![Test Status](https://github.com/rickhull/device_control/actions/workflows/test.yaml/badge.svg)](https://github.com/rickhull/device_control/actions/workflows/test.yaml)

# Rationale

At present, this library scratches the itch of implementing a simple PID
controller.  It builds up to this with some simple abstractions that can
also be used to build other, more sophisticated controllers.

# Concepts

## Controller

A controller is a piece of equipment (or, more abstractly, perhaps even a
human operator) that is intended to achieve a certain measurement from the
environment.  For example, a thermostat wants to maintain a temperature, or
the cruise control in your car wants to maintain a certain wheelspeed.

A thermostat on its own cannot (meaningfully) affect the environment; it is
just a controller, presumably for some other device, like a heater.  The
thermostat, if hooked up to a heating device, can control when the heat
comes on, and this changes the measurement from the environment, ideally
towards the desired temperature.

## Device

I'm not sure about the actual nomenclature from a very rich field of study,
*control theory*, but I'm using the term "device" to describe that which the
controller is controlling.  So a heater or a refrigerator may be the device,
or the throttle on an engine, or a broomstick balanced on a pencil eraser.

A controller requires a device, and a device must have some variable input,
like a control knob, which the controller can thus manipulate.  The device
presumably reacts to the input with a new output, and this output presumably
affects the environment in some way that the controller can measure.

## Environment

The environment, in some way, connects the output of the device back to
the measurement on the controller.  Often, in order to test a device or a
controller (or both), the environment must be modeled or simulated, often
crudely.  Or perhaps the environment is already inherent to the problem, or
it has been modeled extensively as part of the problem.

This project will make little or no effort to model your environment.  But
it's important to recognize that you have to "close the loop" for any of this
to make sense.

# Approach

## Control Loop

Our control loop is composed of the 3 concepts above:

```
CONTROLLER ----> DEVICE
        ^         |
        |         |
        |         V
        ENVIRONMENT
```

### A Pattern

Each component accepts an input and yields an output.  **Controllers** accept
a measure and yield a control value.  **Devices** accept a control value and
yield an environmental output.  The **environment** accepts the new output and
produces a new measure for the controller.

It's worth noting that a control loop may include multiple controllers feeding
one another as well as multiple devices.  And the environment may affect
different stages of the control loop in different ways.

```ruby
module Updateable
  def update(val)
    self.input = val
    self.output
  end
end
```

Notice, this is a module, not a class.  This module is intended to be mixed in
to a class in order provide (and guarantee) the pattern of behavior.  Any
class which wants to mix in `Updateable` should thus, at minimum, define:

* `initialize`
* `input=`
* `output`

Note that the class can use any ivars; there is no need to create or ever
touch `@input` if a different ivar name is preferred.

### Device

```ruby
class Device
  include Updateable

  attr_reader :knob

  def initialize
    @knob = 0.0
  end

  def input=(val)
    @knob = val.to_f
  end
  alias_method :knob=, :input=

  def output
    @knob # do nothing by default
  end

  def to_s
    format("Knob: %.3f\tOutput: %.3f", @knob, self.output)
  end
end
```

We've named our class `Device`, mixed in `Updateable`, and we've named our input
`knob`.  In general, we will operate only on Floats for inputs and outputs,
though perhaps interesting things can be done outside this limitation.

`@knob` is initialized to zero, and `input=(val)` will update `@knob`.  As
this is a generic device, we will just pass along the input as our output.
Let's also make a friendly string output.

#### Heater

```ruby
class Heater < Device
  # convert electricity into thermal output
  EFFICIENCY = 0.999

  attr_reader :watts

  def initialize(watts, threshold: 0)
    super()
    @watts = watts
    @threshold = threshold
  end

  # output is all or none
  def output
    @knob > @threshold ? (@watts * self.class::EFFICIENCY) : 0
  end

  def to_s
    format("Power: %d W\tKnob: %.1f\tThermal: %.1f W",
           @watts, @knob, self.output)
  end
end
```

Starting with a generic device, we'll add `@watts` for output, and we'll also
allow a configurable threshold for the knob -- *at what point does the knob
turn to on?*  By default, anything above 0.

BTW, this is a crude model, as `@watts` sort of represents the input energy,
and we are representing its output as "the amout of heat that 1000 watts
(or whatever) puts out".  Since electric devices waste power by shedding heat,
electric heaters are very efficient by definition.  It's not difficult to dump
all your power into heat; just use a big resistor.

#### Cooler

```ruby
class Cooler < Heater
  # not nearly as efficient as a heater at turning electrons into therms
  EFFICIENCY = 0.35
end
```

A cooler is just a heater that puts out watts of cooling.  You'd have to
model the inverse effect in your environment.  You can of course create more
sophisticated Heater and Cooler models as well ;)

### Controller

```ruby
class Controller
  include Updateable

  attr_reader :measure
  attr_accessor :setpoint

  def initialize(setpoint)
    @setpoint, @measure = setpoint, 0.0
  end

  def input=(val)
    @measure = val.to_f
  end
  alias_method :measure=, :input=

  # just output the error
  def output
    @setpoint - @measure
  end

  def to_s
    format("Setpoint: %.3f\tMeasure: %.3f", @setpoint, @measure)
  end
end
```

A Controller names its input `measure`, and it introduces a `setpoint`, and
the difference between setpoint and measure is the error.

#### Thermostat

```ruby
class Thermostat < Controller
  # true or false; can drive a Heater or a Cooler
  # true means input below setpoint; false otherwise
  def output
    @setpoint - @measure > 0
  end
end
```

Notice, this thermostat essentially answers the question: *is it hot enough?*
(or equivalently: *is it too cold?*).  You can run it either or both ways,
but note that you can simply pick one orientation and remain logically
consistent; consider:

```ruby

h = Heater.new(1000)
ht = Thermostat.new(20)
c = Cooler.new(1000)
ct = Thermostat.new(25)

temp = 26.4

heat_knob = ht.update(temp) ? 1 : 0
heating_watts = h.update(heat_knob)
cool_knob = ct.update(temp) ? 0 : 1
cooling_watts = c.update(cool_knob)

temp = 24.9

# ...

```

So the **heat knob** goes to 1 when its thermostat goes *below* setpoint.
The **cool knob** goes to 1 when its thermostat goes *above* setpoint.

#### Stateful Controller

Now let's make a more sophisticated controller that has a notion of time
as well as tracking its error over time.

```ruby
class StatefulController < Controller
  HZ = 1000
  TICK = Rational(1) / HZ

  attr_accessor :dt
  attr_reader :error, :last_error, :sum_error

  def initialize(setpoint, dt: TICK)
    super(setpoint)
    @dt = dt
    @error, @last_error, @sum_error = 0.0, 0.0, 0.0
  end

  # update @error, @last_error, and @sum_error
  def input=(val)
    @measure = val
    @last_error = @error
    @error = @setpoint - @measure
    if @error * @last_error <= 0  # zero crossing; reset the accumulated error
      @sum_error = @error * @dt
    else
      @sum_error += @error * @dt
    end
  end

  def to_s
    [super,
     format("Error: %+.3f\tLast: %+.3f\tSum: %+.3f",
            @error, @last_error, @sum_error),
    ].join("\n")
  end
end
```

For a notion of time, we will make a timeslice or a tick (or dt), with 1000
ticks per second.  We'll have ivars for the current error, last error, and
accumulated error.  On a new measure, set the last error to the current error,
set the new error, and accumulate the error by the timeslice.

Note that, so far, we aren't doing anything useful with this information.

#### PID Controller

```ruby
class PIDController < StatefulController
  attr_accessor :kp, :ki, :kd, :p_range, :i_range, :d_range, :o_range

  def initialize(setpoint, dt: TICK)
    super

    # gain / multipliers for PID; tunables
    @kp, @ki, @kd = 1.0, 1.0, 1.0

    # optional clamps for PID terms and output
    @p_range = (-Float::INFINITY..Float::INFINITY)
    @i_range = (-Float::INFINITY..Float::INFINITY)
    @d_range = (-Float::INFINITY..Float::INFINITY)
    @o_range = (-Float::INFINITY..Float::INFINITY)

    yield self if block_given?
  end

  def output
    (self.proportion +
     self.integral +
     self.derivative).clamp(@o_range.begin, @o_range.end)
  end

  def proportion
    (@kp * @error).clamp(@p_range.begin, @p_range.end)
  end

  def integral
    (@ki * @sum_error).clamp(@i_range.begin, @i_range.end)
  end

  def derivative
    (@kd * (@error - @last_error) / @dt).clamp(@d_range.begin, @d_range.end)
  end

  def to_s
    [super,
     format(" Gain:\t%.3f\t%.3f\t%.3f",
            @kp, @ki, @kd),
     format("  PID:\t%+.3f\t%+.3f\t%+.3f\t= %.5f",
            self.proportion, self.integral, self.derivative, self.output),
    ].join("\n")
  end
end
```

# Finale

If you've made it this far, congratulations!  For further reading:

* [lib/device_control.rb](lib/device_control.rb)
* [test/device_control.rb](test/device_control.rb)
