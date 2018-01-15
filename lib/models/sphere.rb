require_relative 'link'

class Sphere < Link
  attr_writer :radius, :diameter

  # geometry
  public
  def radius
    return @radius if @radius  # if both are set, radius has priority over diameter
    return @diameter / 2.0 if @diameter
    0.05 # default diameter is 10 cm
  end

  public
  def diameter
    return @radius*2.0  if @radius
    @diameter || 0.1
  end

  private
  def volume
    4 * Math::PI * radius**3 / 3
  end

  # inertia
  public
  def ixx
    (mass/5.0) * 2*radius**2
  end

  public
  def iyy
    ixx
  end

  public
  def izz
    ixx
  end
end
