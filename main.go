package main

import (
	"./boid"
	"./kdtree"
	"encoding/json"
	"fmt"
	"io/ioutil"
	"log"
	"net/http"
	"time"
)

type Parameters struct {
	CenterOfMass  float64
	Separation    float64
	MatchVelocity float64
	TendToPlaceX  float64
	TendToPlaceY  float64
	TendToPlace   float64
}

var defaultParameters = Parameters{
	0.007,
	0.0001,
	0.03,
	0.6, 0.6, 0.0001,
}

var parameters = defaultParameters

var theBoids []*boid.Boid

func updatePositions() {

	for {

		// calculate new velocities
		points := make([]kdtree.Point, len(theBoids))
		for i, v := range theBoids {
			points[i] = kdtree.Point(v)
		}

		kd := kdtree.Build(points)

		for i := range theBoids {
			points := kd.KNearestNeighbour(theBoids[i], 7)

			boids := make([]*boid.Boid, len(points))
			for i, v := range points {
				boids[i] = v.(*boid.Boid)
			}

			theBoids[i].CenterOfMass(boids, parameters.CenterOfMass)
			theBoids[i].Separation(boids, parameters.Separation)
			theBoids[i].MatchVelocity(boids, parameters.MatchVelocity)
			theBoids[i].TendToPlace(parameters.TendToPlaceX,
				parameters.TendToPlaceY,
				parameters.TendToPlace)
		}

		for i := range theBoids {
			theBoids[i].UpdatePosition()
		}

		// run at 50 Hz
		time.Sleep(20 * time.Millisecond)
	}
}

func handleRoot(writer http.ResponseWriter, request *http.Request) {

	bytes, err := ioutil.ReadFile("index.html")
	if err != nil {
		writer.WriteHeader(http.StatusInternalServerError)
	} else {
		fmt.Fprintf(writer, string(bytes))
	}
}

func handleBoids(writer http.ResponseWriter, request *http.Request) {

	bytes, err := json.Marshal(theBoids)

	if err != nil {
		writer.WriteHeader(http.StatusInternalServerError)
	} else {
		fmt.Fprintf(writer, string(bytes))
	}
}

func handleParameters(writer http.ResponseWriter, request *http.Request) {

	if request.Method == "GET" {
		bytes, err := json.Marshal(parameters)

		if err != nil {
			writer.WriteHeader(http.StatusBadRequest)
		} else {
			fmt.Fprintf(writer, string(bytes))
		}
	} else if request.Method == "POST" {
		decoder := json.NewDecoder(request.Body)
		var requestedParameters Parameters
		err := decoder.Decode(&requestedParameters)
		if err != nil {
			writer.WriteHeader(http.StatusInternalServerError)
		}

		parameters.CenterOfMass = requestedParameters.CenterOfMass
		parameters.Separation = requestedParameters.Separation
		parameters.MatchVelocity = requestedParameters.MatchVelocity
		parameters.TendToPlace = requestedParameters.TendToPlace

		writer.WriteHeader(http.StatusOK)
	} else {
		writer.WriteHeader(http.StatusMethodNotAllowed)
	}
}

func handleParametersReset(writer http.ResponseWriter, request *http.Request) {
	parameters = defaultParameters
	writer.WriteHeader(http.StatusOK)
}

func main() {
	theBoids = make([]*boid.Boid, 100)

	// initialize boids
	for i := range theBoids {
		theBoids[i] = new(boid.Boid)
		theBoids[i].Initialize(0.25, 0.25, 0.15)
	}

	go updatePositions()

	http.HandleFunc("/", handleRoot)
	http.HandleFunc("/boids", handleBoids)
	http.HandleFunc("/parameters", handleParameters)
	http.HandleFunc("/parameters/reset", handleParametersReset)
	if err := http.ListenAndServe(":9001", nil); err != nil {
		log.Fatal("failed to start server", err)
	}
}
