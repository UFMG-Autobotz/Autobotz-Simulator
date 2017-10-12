class Kicker
  def initialize(distance, radius, resolution)
    @radius = radius
    @center = Math.sqrt(radius**2 - distance**2)
    @theta = Math.atan2(distance, @center)
    @dtheta = 2* @theta/resolution
  end

  def center
    @center
  end

  def x(i)
    @center - @radius * Math.cos(@theta - i*@dtheta)
  end

  def y(i)
    - @radius * Math.sin(@theta - i*@dtheta)
  end
end
