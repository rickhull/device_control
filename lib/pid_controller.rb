# This is an example of the API pattern for a Controller (e.g. thermostat)
# or a Device (e.g. heater).
#
# They each have an input that varies over time which determines their output.
#
# A thermostat (Controller) listens for  temperature and tells the heater how
# high to turn it up (or just on / off).
#
# A heater (Device) listens to its control knob and yields heat as an output.
#
class Processor
  HZ = 1000
  TICK = Rational(1) / HZ

  attr_reader :input

  def initialize(dt: TICK)
    @dt = dt
  end

  def update(input)
    self.input = input
    self.output
  end

  # presumably overwritten by a subclass; probably update some state
  def input=(val)
    @input = val
  end

  # presumably overwritten by a subclass
  def output
    @input
  end
end

# a Controller's main interface is the update method, which given a measure
# provides an output
class Controller
  HZ = Processor::HZ
  TICK = Processor::TICK

  attr_accessor :setpoint, :dt
  attr_reader :measure, :error, :last_error, :sum_error

  def initialize(setpoint, dt: TICK)
    @setpoint, @dt, @measure = setpoint, dt, 0.0
    @error, @last_error, @sum_error = 0.0, 0.0, 0.0
  end

  # this is the main interface for a controller
  # provide a measure, and get a new setting in response
  def update(measure)
    self.measure = measure
    self.output
  end

  def measure=(val)
    @measure = val
    @last_error = @error
    @error = @setpoint - @measure
    if @error * @last_error <= 0  # zero crossing; reset the accumulated error
      @sum_error = @error
    else
      @sum_error += @error
    end
  end

  # the output will depend on the @measure in some way
  def output
    @error
  end

  def to_s
    [format("Setpoint: %.3f\tMeasure: %.3f\tdt: %s",
            @setpoint, @measure, @dt),
     format("Error: %+.3f\tLast: %+.3f\tSum: %+.3f",
            @error, @last_error, @sum_error),
    ].join("\n")
  end
end

# input is the current ambient temp (monitored for safety cutoff)
# output is watts
class Heater < Processor
  # convert electricity into thermal output
  EFFICIENCY = 0.95

  attr_accessor :cutoff
  attr_reader :watts

  def initialize(watts, dt: TICK)
    super(dt: dt)
    @watts = watts
    @cutoff = 35 # degrees C
  end

  # output is all or none
  def output
    @input < @cutoff ? (@watts * EFFICIENCY) : 0
  end
end

class Cooler < Heater
  # not nearly as efficient as heaters in turning electrons into therms
  EFFICIENCY = 0.35

  def initialize(watts, dt: TICK)
    super
    @cutoff = 10 # degrees C
  end

  # output is all or none
  def output
    @input > @cutoff ? (@watts * EFFICIENCY) : 0
  end
end

class Thermostat < Controller
  # true or false; can drive a Heater or a Cooler
  def output
    @error > 0
  end
end

# now consider e.g.
# h = Heater.new(1000)
# ht = Thermostat.new(20)
# c = Cooler.new(1000)
# ct = Thermostat.new(25)
# temp = 26.4
# heating_watts = h.update(ht.update(temp))
# cooling_watts = c.update(ct.update(temp))
# etc


# Track:
# * current error (setpoint - measure)
# * accumulated error
# * last error
# In order to calculate:
# * Proportion (current error)
# * Integral   (accumulated error)
# * Derivative (error slope, last_error)
#
class PIDController < Controller
  # Ziegler-Nichols method for tuning PID gain knobs
  ZN = {
    #            Kp     Ti     Td     Ki     Kd
    #     Var:   Ku     Tu     Tu    Ku/Tu  Ku*Tu
    'P'  =>   [0.500],
    'PI' =>   [0.450, 0.800,   nil, 0.540],
    'PD' =>   [0.800,   nil, 0.125,   nil, 0.100],
    'PID' =>  [0.600, 0.500, 0.125, 1.200, 0.075],
    'PIR' =>  [0.700, 0.400, 0.150, 1.750, 0.105],
    # less overshoot than standard PID below
    'some' => [0.333, 0.500, 0.333, 0.666, 0.111],
    'none' => [0.200, 0.500, 0.333, 0.400, 0.066],
  }

  # ku = ultimate gain, tu = oscillation period
  def self.tune(type, ku, tu)
    record = ZN[type.downcase] || ZN[type.upcase] || ZN.fetch(type)
    kp, ti, td, ki, kd = *record
    kp *= ku if kp
    ti *= tu if ti
    td *= tu if td
    ki *= (ku / tu) if ki
    kd *= (ku * tu) if kd
    { kp: kp, ti: ti, td: td, ki: ki, kd: kd }
  end

  attr_accessor :kp, :ki, :kd, :p_range, :i_range, :d_range, :o_range
  attr_reader :error, :last_error, :sum_error

  def initialize(setpoint, dt: TICK)
    super(setpoint, dt: dt)

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
    (@ki * @sum_error * @dt).clamp(@i_range.begin, @i_range.end)
  end

  def derivative
    (@kd * (@error - @last_error) / @dt).clamp(@d_range.begin, @d_range.end)
  end

  def to_s
    super +
      [format(" Gain:\t%.3f\t%.3f\t%.3f",
              @kp, @ki, @kd),
       format("  PID:\t%+.3f\t%+.3f\t%+.3f\t= %.5f",
              self.proportion, self.integral, self.derivative, self.output),
      ].join("\n")
  end
end
