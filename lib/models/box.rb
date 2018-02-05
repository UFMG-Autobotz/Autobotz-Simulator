require_relative 'link'

class Box < Link
  attr_accessor :length, :width, :height

  def initialize()
    super
    @length = @width = @height = 0.1 #default is a 10x10x10 cm cube
  end

  # geometry
  private
  def volume
    @length * @width * @height
  end

  # pose
  public
  def z
    @z || height/2
  end

  # inertia
  public
  def ixx
    mass/12.0 * (width**2 + height**2)
  end

  public
  def iyy
    mass/12.0 * (height**2 + length**2)
  end

  public
  def izz
    mass/12.0 * (length**2 + width**2)
  end
end
