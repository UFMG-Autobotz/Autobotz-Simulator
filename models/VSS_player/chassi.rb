lib = '../../lib/models/' # path to simple shape ruby objects
require_relative lib + 'box'
require_relative 'kicker'

class Chassi < Box
  attr_writer :h2, :h3
  attr_accessor :th, :h1
  attr_reader :body, :kicker, :tag

  def initialize
    super
    @body = Box.new
    @kicker = Kicker.new
    @tag = Box.new
  end

  public
  def calc_geometry
    kicker.height = @h2 - @h1
    kicker.calc_geometry(length)

    body.length = length
    body.width = width
    body.height = @h3 - @h2
    body.z = kicker.height + 0.5*body.height*(1-th)

    tag.length = length
    tag.width = width
    tag.height = th*height
    tag.z  = height*(1 - 0.5*th)
  end

  public
  def height
    @h3 - @h1
  end

end
