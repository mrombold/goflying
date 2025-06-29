package ahrsweb

import (
	"encoding/json"
	"log"
	"time"
	//"math"
	"net/url"

	"github.com/stratux/goflying/ahrs"
	"fmt"
	"github.com/gorilla/websocket"
)

type KalmanListener struct {
	data *AHRSData
	c    *websocket.Conn
}

func NewKalmanListener() (kl *KalmanListener, err error) {
	kl = new(KalmanListener)
	kl.data = new(AHRSData)
	if err = kl.connect(); err != nil {
		return nil, err
	}

	return kl, nil
}

func (kl *KalmanListener) connect() (err error) {
	u := url.URL{Scheme: "ws", Host: fmt.Sprintf("localhost:%d", Port), Path: "/ahrsweb"}
	kl.c, _, err = websocket.DefaultDialer.Dial(u.String(), nil)
	return
}

func (kl *KalmanListener) update(s *ahrs.State, m *ahrs.Measurement) {
	kl.data.T = float64(time.Now().UnixNano()/1000) / 1e6

	if s != nil {
		kl.data.U1 = 0
		kl.data.U2 = 0
		kl.data.U3 = 0
		kl.data.Z1 = 0
		kl.data.Z2 = 0
		kl.data.Z3 = 0
		kl.data.E0 = 0
		kl.data.E1 = 0
		kl.data.E2 = 0
		kl.data.E3 = 0
		kl.data.H1 = 0
		kl.data.H2 = 0
		kl.data.H3 = 0
		kl.data.N1 = 0
		kl.data.N2 = 0
		kl.data.N3 = 0

		kl.data.V1 = 0
		kl.data.V2 = 0
		kl.data.V3 = 0
		kl.data.C1 = 0
		kl.data.C2 = 0
		kl.data.C3 = 0
		kl.data.F0 = 0
		kl.data.F1 = 0
		kl.data.F2 = 0
		kl.data.F3 = 0
		kl.data.D1 = 0
		kl.data.D2 = 0
		kl.data.D3 = 0
		kl.data.L1 = 0
		kl.data.L2 = 0
		kl.data.L3 = 0

	} else {
		log.Println("AHRSWeb: state is nil, not updating data")
	}

	if m != nil {
		kl.data.UValid = m.UValid
		kl.data.WValid = m.WValid
		kl.data.SValid = m.SValid
		kl.data.MValid = m.MValid

		kl.data.S1 = 0
		kl.data.S2 = 0
		kl.data.S3 = 0
		kl.data.W1 = 0
		kl.data.W2 = 0
		kl.data.W3 = 0
		kl.data.A1 = m.A1
		kl.data.A2 = m.A2
		kl.data.A3 = m.A3
		kl.data.B1 = m.B1
		kl.data.B2 = m.B2
		kl.data.B3 = m.B3
		kl.data.M1 = m.M1
		kl.data.M2 = m.M2
		kl.data.M3 = m.M3
	} else {
		log.Println("AHRSWeb: measurement is nil, not updating data")
	}
}

func (kl *KalmanListener) Send(s *ahrs.State, m *ahrs.Measurement) error	 {
	kl.update(s, m)

	if msg, err := json.Marshal(kl.data); err != nil {
		log.Println("AHRSWeb: Error marshalling json data:", err)
		log.Println("AHRSWeb: Data was:", kl.data)
		return err
	} else {
		if err := kl.c.WriteMessage(websocket.TextMessage, msg); err != nil {
			log.Println("AHRSWeb: Error writing to websocket:", err)
			err2 := kl.connect()
			return fmt.Errorf("AHRSWeb: %v: %v", err, err2) // Just drop this message
		}
	}
	return nil
}

func (kl *KalmanListener) Close() {
	if err := kl.c.WriteMessage(websocket.CloseMessage, websocket.FormatCloseMessage(websocket.CloseNormalClosure, "")); err != nil {
		log.Println("AHRSWeb: Error closing websocket:", err)
		return
	}
	kl.c.Close()
}
