package main

import (
	"encoding/json"
	"log"
	"net/http"

	"github.com/gorilla/mux"
	"github.com/rs/cors"
)

var waypoints = make([][]float64, 0)

// Handler for POST /waypoints
func updateWaypointsHandler(w http.ResponseWriter, r *http.Request) {

	waypoints = make([][]float64, 0)

	var reqBody struct {
		Waypoints [][]float64 `json:"waypoints"`
	}

	if err := json.NewDecoder(r.Body).Decode(&reqBody); err != nil {
		http.Error(w, `{"message": "Invalid request body"}`, http.StatusBadRequest)
		return
	}

	if len(reqBody.Waypoints) != 0 {
		for _, coords := range reqBody.Waypoints {
			if len(coords) != 2 {
				http.Error(w, `{"message": "Each waypoint must be [lat, lon]"}`, http.StatusBadRequest)
				return
			}
			wp := make([]float64, 2)
			wp[0] = coords[0]
			wp[1] = coords[1]
			waypoints = append(waypoints, wp)
		}
	}

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]string{"message": "Waypoints added successfully"})
}

// Handler for GET /waypoints
func getWaypointsHandler(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(waypoints)
}

func main() {
	router := mux.NewRouter()

	router.HandleFunc("/waypoints", updateWaypointsHandler).Methods("POST")
	router.HandleFunc("/waypoints", getWaypointsHandler).Methods("GET")

	handler := cors.Default().Handler(router)

	port := ":3001"
	log.Printf("Server running at http://localhost%s\n", port)
	if err := http.ListenAndServe(port, handler); err != nil {
		log.Fatalf("Could not start server: %s\n", err)
	}
}
