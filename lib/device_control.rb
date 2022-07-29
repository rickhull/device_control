module DeviceControl
  # There is a pattern for how both Controllers (e.g. thermostat) and Devices
  #   (e.g. heater) operate.
  # They each have an _input_ varying over time which determines the _output_.
  # A thermostat (Controller) listens for  temperature and tells the heater how
  # high to turn it up (or just on / off).  A heater (Device) listens to its
  # control knob and yields heat as an output.
  #
  # We capture this pattern with a single method: _update_.  It accepts the
  # latest input and provides an _output_ based on the input. When the input
  # is read in, perhaps some internal state is changed on
  # the processor which will affect the _output_.
  #
  # Any class which mixes in Updateable can define its own _input=_ method,
  # which may update any ivars.  Any such class must define an _output_ method.
  #
  module Updateable
    def update(val)
      self.input = val
      self.output
    end
  end

  # A Device is like a heater.  It has a control knob, maybe on/off or perhaps
  # a variable control.  Its output (maybe on/off) depends on the control knob.
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

  # Alright, fine, let's make a Heater
  # Input is the control knob (turned far enough to on, else off)
  # Output is watts
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

  class Cooler < Heater
    # not nearly as efficient as a heater at turning electrons into therms
    EFFICIENCY = 0.35
  end

  # A Controller is like a thermostat.  It has a setpoint, and it reads a
  # measurement from the environment, and it adjusts its output to try to make
  # the measurement match the setpoint.
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

  class Thermostat < Controller
    # true or false; can drive a Heater or a Cooler
    # true means input below setpoint; false otherwise
    def output
      @setpoint - @measure > 0
    end
  end

  # now consider e.g.
  # h = Heater.new(1000)
  # ht = Thermostat.new(20)
  # c = Cooler.new(1000)
  # ct = Thermostat.new(25)
  # temp = 26.4
  # heat_knob = ht.update(temp) ? 1 : 0
  # heating_watts = h.update(heat_knob)
  # cool_knob = ct.update(temp) ? 0 : 1
  # cooling_watts = c.update(cool_knob)
  # etc

  class Flexstat < Thermostat
    def self.cold_val(hot_val)
      case hot_val
      when true, false
        !hot_val
      when 0,1
        hot_val == 0 ? 1 : 0
      when Numeric
        0
      when :on, :off
        hot_val == :on ? :off : :on
      else
        raise "#{hot_val.inspect} not recognized"
      end
    end

    attr_reader :cold_val, :hot_val

    def initialize(setpoint, hot_val: false, cold_val: nil)
      super(setpoint)

      @hot_val = hot_val
      @cold_val = cold_val.nil? ? self.class.cold_val(hot_val) : cold_val
    end

    def output
      super ? @cold_val : @hot_val
    end
  end

  # A PIDController is a Controller that tracks its error over time
  # in order to calculate:
  #   Proportion (current error)
  #   Integral   (accumulated error)
  #   Derivative (error slope, last_error)
  # The sum of these terms is the output
  #
  class PIDController < Controller
    HZ = 1000
    TICK = Rational(1) / HZ

    # Ziegler-Nichols method for tuning PID gain knobs
    # https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
    ZN = {
      #           Kp     Ti    Td     Ki     Kd
      #     Var:  Ku     Tu    Tu    Ku/Tu  Ku*Tu
      'P'    => [1/2r],
      'PI'   => [9/20r, 4/5r,   nil, 27/50r],
      'PD'   => [ 4/5r,  nil,  1/8r,  nil, 1/10r],
      'PID'  => [ 3/5r, 1/2r,  1/8r, 6/5r, 3/40r],
      'PIR'  => [7/10r, 2/5r, 3/20r, 7/4r, 21/200r],
      # less overshoot than standard PID
      'some' => [ 1/3r, 1/2r,  1/3r, 2/3r, 1/11r],
      'none' => [ 1/5r, 1/2r,  1/3r, 2/5r, 2/30r],
    }

    # _ku_ = ultimate gain, _tu_ = oscillation period
    # output includes ti and td, which are not necessary
    # typically kp, ki, and kd are used
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

    attr_accessor :dt, :low_pass_ticks,
                  :error, :last_error, :sum_error,
                  :kp, :ki, :kd,
                  :p_range, :i_range, :d_range, :o_range, :e_range
    attr_reader :mavg

    def initialize(setpoint, dt: TICK, low_pass_ticks: 0)
      super(setpoint)
      @dt = dt
      @error, @last_error, @sum_error = 0.0, 0.0, 0.0
      if low_pass_ticks > 0
        @mavg = MovingAverage.new(low_pass_ticks)
      else
        @mavg = nil
      end

      # gain / multipliers for PID; tunables
      @kp, @ki, @kd = 1.0, 1.0, 1.0

      # optional clamps for PID terms and output
      @p_range = (-Float::INFINITY..Float::INFINITY)
      @i_range = (-Float::INFINITY..Float::INFINITY)
      @d_range = (-Float::INFINITY..Float::INFINITY)
      @o_range = (-Float::INFINITY..Float::INFINITY)
      @e_range = (-Float::INFINITY..Float::INFINITY)

      yield self if block_given?
    end

    # update @error, @last_error, and @sum_error
    def input=(val)
      @measure = val
      @last_error = @error
      @error = @setpoint - @measure
      @sum_error =
        (@sum_error + @ki * @error * @dt).clamp(@e_range.begin, @e_range.end)
      @mavg.input = self.derivative if @mavg
    end

    def output
      drv = @mavg ? @mavg.output : self.derivative
      (self.proportion +
       self.integral +
       drv).clamp(@o_range.begin, @o_range.end)
    end

    def proportion
      (@kp * @error).clamp(@p_range.begin, @p_range.end)
    end

    def integral
      @sum_error.clamp(@i_range.begin, @i_range.end)
    end

    def derivative
      (@kd * (@error - @last_error) / @dt).clamp(@d_range.begin, @d_range.end)
    end

    def to_s
      [super,
       format("Error: %+.3f\tLast: %+.3f\tSum: %+.3f",
              @error, @last_error, @sum_error),
       format(" Gain:\t%.3f\t%.3f\t%.3f",
              @kp, @ki, @kd),
       format("  PID:\t%+.3f\t%+.3f\t%+.3f\t= %.5f",
              self.proportion, self.integral, self.derivative, self.output),
      ].join("\n")
    end
  end

  class MovingAverage
    include Updateable

    def initialize(size = 2)
      @size = size
      @idx = 0
      @storage = Array.new(@size, 0)
    end

    def input=(val)
      @storage[@idx % @size] = val
      @idx += 1
    end

    def output
      return 0 if @idx == 0
      @storage.sum / (@idx > @size ? @size : @idx).to_f
    end
  end

  class RateLimiter
    include Updateable

    def initialize(max_step)
      @max_step = max_step
      @val = 0
    end

    def input=(val)
      diff = val - @val
      @val += diff.clamp(-1 * @max_step, @max_step)
    end

    def output
      @val
    end
  end
end
