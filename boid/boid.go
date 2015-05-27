package boid

import (
	"math/rand"
)

type Boid struct {
	X, Y         float64
	X_dot, Y_dot float64
}

func (self *Boid) Initialize(x, y, sigma float64) {
	self.X = rand.NormFloat64()*sigma + x
	self.Y = rand.NormFloat64()*sigma + y
	self.X_dot = (1.0 - 2.0*rand.Float64()) * 0.01
	self.Y_dot = (1.0 - 2.0*rand.Float64()) * 0.01
}

func (self *Boid) CenterOfMass(boids []*Boid, weight float64) {

	center_of_mass_X := 0.0
	center_of_mass_Y := 0.0
	for i := range boids {
		center_of_mass_X += boids[i].X
		center_of_mass_Y += boids[i].Y
	}

	self.X_dot += ((center_of_mass_X / float64(len(boids))) - self.X) * weight
	self.Y_dot += ((center_of_mass_Y / float64(len(boids))) - self.Y) * weight
}

func (self *Boid) Separation(boids []*Boid, threshold float64) {

	separation_X := 0.0
	separation_Y := 0.0
	for i := range boids {

		delta_X := boids[i].X - self.X
		delta_Y := boids[i].Y - self.Y
		if (delta_X*delta_X + delta_Y*delta_Y) < threshold {
			separation_X -= delta_X
			separation_Y -= delta_Y
		}
	}

	self.X_dot += separation_X
	self.Y_dot += separation_Y
}

func (self *Boid) MatchVelocity(boids []*Boid, weight float64) {
	match_X_dot := 0.0
	match_Y_dot := 0.0

	for i := range boids {
		match_X_dot += boids[i].X_dot
		match_Y_dot += boids[i].Y_dot
	}

	self.X_dot += (match_X_dot / float64(len(boids))) * weight
	self.Y_dot += (match_Y_dot / float64(len(boids))) * weight
}

func (self *Boid) TendToPlace(x, y, weight float64) {

	self.X_dot += (x - self.X) * weight
	self.Y_dot += (y - self.Y) * weight
}

func (self *Boid) UpdatePosition() {

	// limit velocitY
	if self.X_dot < -0.005 {
		self.X_dot = -0.005
	}

	if self.Y_dot < -0.005 {
		self.Y_dot = -0.005
	}

	if self.X_dot > 0.005 {
		self.X_dot = 0.005
	}

	if self.Y_dot > 0.005 {
		self.Y_dot = 0.005
	}

	self.X += self.X_dot
	self.Y += self.Y_dot
}
