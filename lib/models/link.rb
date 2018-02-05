class Link
  attr_writer :mass, :density, :mu, :mu2
  attr_writer :x, :y, :z
  attr_writer :roll, :pitch, :yaw

  def initialize()
    @density = 1000 #default density is 1000 kg/m³
  end

  # pose
  public
  def x
    @x || 0
  end

  public
  def y
    @y || 0
  end

  public
  def z
    @z || 0
  end

  public
  def roll
    @roll || 0
  end

  public
  def picht
    @picht || 0
  end

  public
  def yaw
    @yaw || 0
  end

  # inertia
  public
  def mass
    @mass || @density * volume # if both are set, mass has priority over density
  end

  # surface
  public
  def mu
    @mu || 0
  end

  public
  def mu2
    @mu2 || @mu || 0
  end
end
