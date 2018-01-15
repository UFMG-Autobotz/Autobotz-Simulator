require_relative 'link'

class Cilinder < Link
  attr_writer :radius, :diameter
  attr_accessor :length

  def initialize()
    super
    @length = 0.1 #default length is 10 cm
  end

  # geometry
  public
  def radius
    return @radius if @radius  # if both are set, radius has priority over diameter
    return @diameter / 2.0 if @diameter
    0.05 # default diameter is 10 cm
  end

  public
  def diameter
    return @radius*2.0 if @radius
    @diameter || 0.1
  end

  private
  def volume
    Math::PI * radius**2 * length
  end

  # inertia
  public
  def ixx
    (mass/12.0) * (3*radius**2 + length**2)
  end

  public
  def iyy
    ixx
  end

  public
  def izz
    (mass/2.0) * radius**2
  end
end
