require 'device_control'
require 'minitest/autorun'

include DeviceControl

# create a basic class that includes Updateable as a Mixin
# the class should define #initialize, #input= and #output at minimum
class Doubler
  include Updateable

  attr_accessor :input

  def initialize
    @input = 0.0
  end

  def output
    @input * 2
  end
end

describe Updateable do
  describe "a mixin that provides an 'update' pattern" do
    before do
      @o = Doubler.new
    end

    it "has an update method that accepts an input and returns output" do
      expect(@o.input).must_equal 0.0
      expect(@o.output).must_equal 0.0

      output = @o.update(45)
      expect(@o.input).must_equal 45
      expect(@o.output).must_equal output
    end

    it "requires an output method" do
      k = Class.new(Object) do
        include Updateable
      end
      o = k.new
      expect { o.update(45) }.must_raise NoMethodError
    end
  end
end

describe Device do
  before do
    @device = Device.new
  end

  it "has an output" do
    expect(@device.output).must_be_kind_of Float
  end

  it "has a string representation" do
    expect(@device.to_s).must_be_kind_of String
  end

  it "has an update method from Updateable" do
    expect(@device.update(2.34)).must_be_kind_of Float
  end
end

describe Heater do
  before do
    @h = Heater.new(1000)
  end

  it "has an output when knob is greater than zero" do
    expect(@h.knob).must_equal 0
    expect(@h.output).must_equal 0
    @h.knob = 1
    expect(@h.output).must_be :>, 0
  end

  it "has a string representation" do
    expect(@h.to_s).must_be_kind_of String
  end

  it "has update from Updateable" do
    expect(@h.knob).must_equal 0
    expect(@h.output).must_equal 0
    output = @h.update(1)
    expect(output).must_be :>, 0
    expect(@h.knob).must_equal 1
    expect(@h.output).must_equal output
  end

  describe Cooler do
    it "is less efficient than a Heater" do
      h = Heater.new(1000)
      c = Cooler.new(1000)
      h.knob = 1
      c.knob = 1
      expect(c.output).must_be :<, h.output
      expect(c.output).must_be :<, h.output / 2.0
    end
  end
end

describe Controller do
  before do
    @sp = 500
    @c = Controller.new(@sp)
  end

  it "has an output_, the difference between setpoint and measure" do
    expect(@c.output).must_be_kind_of Float
    expect(@c.output).must_equal @sp
  end

  it "has a string representation" do
    expect(@c.to_s).must_be_kind_of String
  end

  it "has an update method from Updateable" do
    expect(@c.update(499)).must_equal 1.0
  end
end

describe Thermostat do
  before do
    @t = Thermostat.new 25
  end

  it "outputs true when it's too cold; when measure < setpoint" do
    expect(@t.update 20).must_equal true
    expect(@t.update 30).must_equal false
  end

  it "outputs false when it's too hot; when measure > setpoint" do
    expect(@t.update 30).must_equal false
    expect(@t.update 20).must_equal true
  end

  describe Flexstat do
    it "maps a hot_val to a suitable cold_val" do
      expect(Flexstat.cold_val true).must_equal false
      expect(Flexstat.cold_val false).must_equal true
      expect(Flexstat.cold_val 0).must_equal 1
      expect(Flexstat.cold_val 1).must_equal 0
      expect(Flexstat.cold_val 2).must_equal 0
      expect(Flexstat.cold_val 99.876).must_equal 0
      expect(Flexstat.cold_val :on).must_equal :off
      expect(Flexstat.cold_val :off).must_equal :on
      expect { Flexstat.cold_val 'on' }.must_raise
      expect { Flexstat.cold_val :anything_else }.must_raise
    end

    it "has configurable hot/cold values for its output" do
      f = Flexstat.new(25, hot_val: :on, cold_val: :off)
      expect(f).must_be_kind_of Flexstat
      expect(f.hot_val).must_equal :on
      expect(f.cold_val).must_equal :off
      expect(f.update 20).must_equal :off

      f = Flexstat.new(25, hot_val: :on)
      expect(f).must_be_kind_of Flexstat
      expect(f.hot_val).must_equal :on
      expect(f.cold_val).must_equal :off
      expect(f.update 30).must_equal :on

      f = Flexstat.new(25)
      expect(f).must_be_kind_of Flexstat
      expect(f.hot_val).must_equal false # default, subject to change
      expect(f.cold_val).must_equal true # likewise
      expect(f.cold_val).must_equal Flexstat.cold_val(f.hot_val)
      expect(f.update 30).must_equal f.hot_val

      f = Flexstat.new(25, hot_val: 7, cold_val: 19)
      expect(f).must_be_kind_of Flexstat
      expect(f.hot_val).must_equal 7
      expect(f.cold_val).must_equal 19
      expect(f.update 20).must_equal 19

      f = Flexstat.new(25, hot_val: 7)
      expect(f).must_be_kind_of Flexstat
      expect(f.hot_val).must_equal 7
      expect(f.cold_val).must_equal Flexstat.cold_val(f.hot_val)
      expect(f.cold_val).must_equal 0
      expect(f.update 30).must_equal 7
    end
  end
end

describe PIDController do
  it "informs Ziegler-Nichols tuning" do
    # P only, not PID
    hsh = PIDController.tune('P', 5, 0.01)
    expect(hsh[:kp]).must_be :>, 0
    expect(hsh[:ki]).must_be_nil
    expect(hsh[:kd]).must_be_nil
    expect(hsh[:ti]).must_be_nil
    expect(hsh[:td]).must_be_nil

    hsh = PIDController.tune('PI', 5, 0.01)
    expect(hsh[:kp]).must_be :>, 0
    expect(hsh[:ki]).must_be :>, 0
    expect(hsh[:kd]).must_be_nil
    expect(hsh[:ti]).must_be :>, 0
    expect(hsh[:td]).must_be_nil

    hsh = PIDController.tune('PID', 5, 0.01)
    expect(hsh[:kp]).must_be :>, 0
    expect(hsh[:ki]).must_be :>, 0
    expect(hsh[:kd]).must_be :>, 0
    expect(hsh[:ti]).must_be :>, 0
    expect(hsh[:td]).must_be :>, 0
  end

  it "has an optional dt argument to initialize" do
    pid = PIDController.new(1000, dt: 0.1)
    expect(pid).must_be_kind_of PIDController
    expect(pid.setpoint).must_equal 1000
    expect(pid.dt).must_equal 0.1
  end

  it "tracks error, last_error, sum_error" do
    pid = PIDController.new(100)
    expect(pid.error).must_equal 0.0
    expect(pid.last_error).must_equal 0.0
    expect(pid.sum_error).must_equal 0.0

    output = pid.update 50
    expect(pid.output).must_equal output
    expect(pid.measure).must_equal 50
    expect(pid.error).must_be_within_epsilon 50.0
    expect(pid.last_error).must_equal 0.0
    expect(pid.sum_error).must_be_within_epsilon(50.0 * pid.dt)

    output = pid.update 75
    expect(pid.output).must_equal output
    expect(pid.measure).must_equal 75
    expect(pid.error).must_be_within_epsilon 25.0
    expect(pid.last_error).must_be_within_epsilon 50.0
    expect(pid.sum_error).must_be_within_epsilon(75.0 * pid.dt)
  end

  it "has PID gain settings" do
    pid = PIDController.new(1000)
    expect(pid.kp).must_be :>, 0
    pid.kp = 1000
    expect(pid.kp).must_equal 1000
    pid.ki = 1000
    expect(pid.ki).must_equal 1000
    pid.kd = 1000
    expect(pid.kd).must_equal 1000
  end

  it "clamps the proportion term" do
    pid = PIDController.new(1000)
    pid.p_range = (0..1)
    pid.update(500)
    expect(pid.proportion).must_equal 1.0
    pid.update(1500)
    expect(pid.proportion).must_equal 0.0
  end

  it "clamps the integral term" do
    pid = PIDController.new(1000)
    pid.i_range = (-1.0 .. 1.0)
    pid.setpoint = 10_000
    pid.update(500)
    expect(pid.integral).must_equal 1.0
    pid.update(10_001)
    pid.update(20_000)
    pid.update(30_000)
    expect(pid.integral).must_equal(-1.0)
  end

  it "clamps the derivative term" do
    pid = PIDController.new(1000)
    pid.d_range = (-1.0 .. 0.0)
    pid.update(0)
    pid.update(10)
    expect(pid.derivative).must_equal(-1.0)
    pid.update(990)
    expect(pid.derivative).must_equal(-1.0)
    pid.update(1000)
    pid.update(990)
    expect(pid.derivative).must_equal(0.0)
  end

  it "clamps the output" do
    pid = PIDController.new(1000)
    pid.o_range = (0.0 .. 1.0)
    pid.update(0)
    expect(pid.output).must_equal(1.0)
    pid.update(2000)
    expect(pid.output).must_equal(0.0)
  end

  it "clamps sum_error" do
    pid = PIDController.new(1000)
    pid.e_range = 999..1000

    pid.update(500)
    expect(pid.error).must_equal 500
    expect(pid.sum_error).must_equal 999
    pid.update(1000)
    expect(pid.sum_error).must_equal 999
    pid.update(-1000)
    expect(pid.sum_error).must_equal 1000
  end

  it "calculates proportion based on current error" do
    pid = PIDController.new(1000)
    pid.kp = 1.0
    pid.update(0)
    expect(pid.proportion).must_equal 1000.0
    pid.update(1)
    expect(pid.proportion).must_equal 999.0
    pid.update(1001)
    expect(pid.proportion).must_equal(-1.0)
  end

  it "calculates integral based on accumulated error" do
    pid = PIDController.new(1000)
    pid.ki = 1.0

    pid.update(0) # error is 1000; dt is 0.001
    expect(pid.integral).must_equal(1.0)

    pid.update(999) # error is 1, sum_error is 1.001
    expect(pid.integral).must_be_within_epsilon(1.001)

    pid.update(1100) # error is -100, sum_error is 0.901
    expect(pid.integral).must_be_within_epsilon(0.901)
  end

  it "calculates derivative based on error slope" do
    pid = PIDController.new(1000)
    pid.kp = 1.0
    pid.update(0)
    expect(pid.error).must_equal 1000
    expect(pid.last_error).must_equal 0
    expect(pid.derivative).must_be_within_epsilon(1000 / pid.dt)

    pid.update(500)
    expect(pid.error).must_equal 500
    expect(pid.last_error).must_equal 1000
    expect(pid.derivative).must_be_within_epsilon(-500 / pid.dt)

    pid.update(999)
    expect(pid.error).must_equal 1
    expect(pid.last_error).must_equal 500
    expect(pid.derivative).must_be_within_epsilon(-499 / pid.dt)

    pid.update(1001)
    expect(pid.error).must_equal(-1)
    expect(pid.last_error).must_equal(1)
    expect(pid.derivative).must_be_within_epsilon(-2 / pid.dt)

    pid.update(1100)
    expect(pid.error).must_equal(-100)
    expect(pid.last_error).must_equal(-1)
    expect(pid.derivative).must_be_within_epsilon(-99 / pid.dt)

    pid.update(900)
    expect(pid.error).must_equal(100)
    expect(pid.last_error).must_equal(-100)
    expect(pid.derivative).must_be_within_epsilon(200 / pid.dt)
  end

  it "has an optional LPF on the derivative term at output" do
    pid = PIDController.new(1000, low_pass_ticks: 2)
    pid.kp = 1.0

    pid.update(0) # error should be 1000; last_error 0 (1000)
    expect(pid.error).must_be_within_epsilon(1000)
    expect(pid.last_error).must_equal 0
    expect(pid.derivative).must_be_within_epsilon(1000 / pid.dt)

    pid.update(500) # error 500, last_error 1000 (-500)
    expect(pid.error).must_equal 500
    expect(pid.last_error).must_equal 1000
    expect(pid.derivative).must_be_within_epsilon(-500 / pid.dt)

    pid.update(999)
    expect(pid.error).must_equal 1
    expect(pid.last_error).must_equal 500
    expect(pid.derivative).must_be_within_epsilon(-499 / pid.dt)

    pid.update(1001)
    expect(pid.error).must_equal(-1)
    expect(pid.last_error).must_equal(1)
    expect(pid.derivative).must_be_within_epsilon(-2 / pid.dt)

    pid.update(1100)
    expect(pid.error).must_equal(-100)
    expect(pid.last_error).must_equal(-1)
    expect(pid.derivative).must_be_within_epsilon(-99 / pid.dt)

    pid.update(900)
    expect(pid.error).must_equal(100)
    expect(pid.last_error).must_equal(-100)
    expect(pid.derivative).must_be_within_epsilon(200 / pid.dt)
  end

  describe MovingAverage do
    it "has a configurable number of items to consider for the average" do
      mavg = MovingAverage.new(5)
      expect(mavg.output).must_equal 0
      expect(mavg.update 10).must_equal 10
      expect(mavg.update 20).must_equal 15
      expect(mavg.update 30).must_equal 20
      expect(mavg.update 20).must_equal 20
      expect(mavg.update 20).must_equal 20

      # 10 is about to drop; replace it to maintain 20
      expect(mavg.update 10).must_equal 20

      # 20 is about to drop; replace it
      expect(mavg.update 20).must_equal 20

      # 30 is about to drop; replace it
      expect(mavg.update 30).must_equal 20

      # 20 is about to drop; replace with 0
      expect(mavg.update 0).must_equal 16
    end
  end

  describe RateLimiter do
    it "has a configurable step size per tick" do
      rl = RateLimiter.new(10)

      expect(rl.update 5).must_equal 5
      expect(rl.update 15).must_equal 15

      # now at 15, attempt 30; limited to 25
      expect(rl.update 30).must_equal 25

      # now at 25, attempt 0; limited to 15
      expect(rl.update 0).must_equal 15

      rl = RateLimiter.new(0.1)
      expect(rl.output).must_equal 0
      expect(rl.update 5).must_be_within_epsilon 0.1
      expect(rl.update 15).must_be_within_epsilon 0.2
    end
  end
end
