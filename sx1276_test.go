package sx1276

import (
	"context"
	"encoding/binary"
	"os"
	"os/signal"
	"testing"
	"time"

	"golang.org/x/xerrors"
)

const (
	Frf = 910e6
)

func TestRx(t *testing.T) {
	sx, err := NewSX1276()
	if err != nil {
		t.Fatalf("%+v\n", xerrors.Errorf("NewSX1276: %w", err))
	}
	defer sx.Close()

	sx.WriteReg(RegLoRaOPMODE, 0|
		LORA_OPMODE_LONGRANGEMODE_ON|
		LORA_OPMODE_SLEEP,
	)

	sx.WriteReg(RegLoRaMODEMCONFIG2, 0|
		LORA_MODEMCONFIG2_SF_7,
	)

	sx.WriteReg(LORA_PAYLOADMAXLENGTH, 0x80)

	sx.SetFreq(Frf)

	sigCh := make(chan os.Signal)
	signal.Notify(sigCh, os.Interrupt, os.Kill)

	ctx := context.Background()
	ctx, cancel := context.WithTimeout(ctx, 5*time.Minute)
	defer cancel()

	pkts := sx.StartRX(ctx)

	t.Log("listening...")

	for {
		select {
		case <-sigCh:
			t.Log("interrupted...")
			return
		case <-ctx.Done():
			t.Log("context cancelled...")
			return
		case pkt := <-pkts:
			var tempU16 uint16

			switch len(pkt) {
			case 44:
				tempU16 = binary.LittleEndian.Uint16(pkt[42:44])
			case 11:
				tempU16 = binary.LittleEndian.Uint16(pkt[9:11])
			}

			temp := (float64(tempU16)-273.15)*1.8 + 32
			rssi := sx.PktRSSI()
			snr := sx.PktSNR()
			fei := sx.PktFEI(125)

			ppm := fei * (10e6 / Frf)

			t.Logf("%02X %0.1fF RSSI:%0.0f SNR:%0.2f FEI:%0.0f PPM:%0.1f\n", pkt,
				temp, rssi, snr, fei, ppm,
			)
		}
	}
}
