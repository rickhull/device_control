require 'pid_controller'
require 'minitest/autorun'

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
    pid.update(0)
    # sum error should be 1000; dt is 0.001
    expect(pid.integral).must_equal(1.0)
    pid.update(999)
    expect(pid.integral).must_be_within_epsilon(1.001)
    pid.update(1100) # zero crossing
    expect(pid.integral).must_be_within_epsilon(-0.1)
  end

  it "calculates derivative based on error slope" do
    pid = PIDController.new(1000)
    pid.kp = 1.0
    pid.update(0)
    # error should be 1000; last_error 0
    expect(pid.derivative).must_equal(1_000_000)
    pid.update(500)
    expect(pid.derivative).must_equal(-500_000)
    pid.update(999)
    expect(pid.derivative).must_equal(-499_000)
    pid.update(1001)
    expect(pid.derivative).must_equal(-2000)
    pid.update(1100)
    expect(pid.derivative).must_equal(-99_000)
    pid.update(900)
    expect(pid.derivative).must_equal(200_000)
  end
end
