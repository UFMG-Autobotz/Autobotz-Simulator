class Link
  attr_writer :mass, :density, :mu, :mu2

  def initialize()
    @density = 1000 #default density is 1000 kg/m³
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
