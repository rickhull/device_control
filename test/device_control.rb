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
  describe "a mixin that provides the _update_ pattern" do
    before do
      @o = Doubler.new
    end

    it "has an _update_ method that accepts an _input_ and returns _output_" do
      expect(@o.input).must_equal 0.0
      expect(@o.output).must_equal 0.0

      output = @o.update(45)
      expect(@o.input).must_equal 45
      expect(@o.output).must_equal output
    end

    it "requires an _output_ method" do
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

  it "has an _output_" do
    expect(@device.output).must_be_kind_of Float
  end

  it "has a string representation" do
    expect(@device.to_s).must_be_kind_of String
  end

  it "has an _update_ method from Updateable" do
    expect(@device.update(2.34)).must_be_kind_of Float
  end
end

describe Heater do
  before do
    @h = Heater.new(1000)
  end

  it "has an _output_ when _knob_ is greater than zero" do
    expect(@h.knob).must_equal 0
    expect(@h.output).must_equal 0
    @h.knob = 1
    expect(@h.output).must_be :>, 0
  end

  it "has a string representation" do
    expect(@h.to_s).must_be_kind_of String
  end

  it "has _update_ from Updateable" do
    expect(@h.knob).must_equal 0
    expect(@h.output).must_equal 0
    output = @h.update(1)
    expect(output).must_be :>, 0
    expect(@h.knob).must_equal 1
    expect(@h.output).must_equal output
  end
end

describe Controller do
  before do
    @sp = 500
    @c = Controller.new(@sp)
  end

  it "has an _output_, the difference between setpoint and measure" do
    expect(@c.output).must_be_kind_of Float
    expect(@c.output).must_equal @sp
  end

  it "has a string representation" do
    expect(@c.to_s).must_be_kind_of String
  end

  it "has an _update_ method from Updateable" do
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

  it "has an optional _dt_ argument to initialize" do
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

  it "clamps the _proportion_ term" do
    pid = PIDController.new(1000)
    pid.p_range = (0..1)
    pid.update(500)
    expect(pid.proportion).must_equal 1.0
    pid.update(1500)
    expect(pid.proportion).must_equal 0.0
  end

  it "clamps the _integral_ term" do
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

  it "clamps the _derivative_ term" do
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

  it "clamps the _output_" do
    pid = PIDController.new(1000)
    pid.o_range = (0.0 .. 1.0)
    pid.update(0)
    expect(pid.output).must_equal(1.0)
    pid.update(2000)
    expect(pid.output).must_equal(0.0)
  end

  it "clamps _sum_error_" do
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

  it "calculates _proportion_ based on current error" do
    pid = PIDController.new(1000)
    pid.kp = 1.0
    pid.update(0)
    expect(pid.proportion).must_equal 1000.0
    pid.update(1)
    expect(pid.proportion).must_equal 999.0
    pid.update(1001)
    expect(pid.proportion).must_equal(-1.0)
  end

  it "calculates _integral_ based on accumulated error" do
    pid = PIDController.new(1000)
    pid.ki = 1.0

    pid.update(0) # error is 1000; dt is 0.001
    expect(pid.integral).must_equal(1.0)

    pid.update(999) # error is 1, sum_error is 1.001
    expect(pid.integral).must_be_within_epsilon(1.001)

    pid.update(1100) # error is -100, sum_error is 0.901
    expect(pid.integral).must_be_within_epsilon(0.901)
  end

  it "calculates _derivative_ based on error slope" do
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

  it "has an optional LPF on the _derivative_ term at _output_" do
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
end
