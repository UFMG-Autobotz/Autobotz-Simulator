lib = '../../lib/models/' # path to simple shape ruby objects
require_relative lib + 'box'

class Kicker < Box
  attr_accessor :height
  attr_reader :radius, :center, :resolution
  attr_writer :dy, :radius, :resolution

  def calc_geometry(chassi_length)
    @distance = 0.5*chassi_length- @dy
    @center = Math.sqrt(@radius**2 - @distance**2)
    @theta = Math.atan2(@distance, @center)
    @dtheta = 2* @theta/resolution
  end

  def x(i)
    @center - @radius * Math.cos(@theta - i*@dtheta)
  end

  def y(i)
    - @radius * Math.sin(@theta - i*@dtheta)
  end
end
