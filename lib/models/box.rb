require_relative 'link'

class Box < Link
  attr_accessor :x, :y, :z

  def initialize()
    super
    @x = @y = @z = 0.1 #default is a 10x10x10 cm cube
  end

  # geometry
  private
  def volume
    @x * @y *@z
  end

  # inertia
  public
  def ixx
    mass/12.0 * (y**2 + z**2)
  end

  public
  def iyy
    mass/12.0 * (z**2 + x**2)
  end

  public
  def izz
    mass/12.0 * (x**2 + y**2)
  end
end
