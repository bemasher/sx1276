package sx1276

import (
	"context"
	"errors"
	"log"
	"time"

	"golang.org/x/xerrors"
	"periph.io/x/periph/conn/gpio"
	"periph.io/x/periph/conn/physic"
	"periph.io/x/periph/conn/spi"
	"periph.io/x/periph/conn/spi/spireg"
	"periph.io/x/periph/host"
	"periph.io/x/periph/host/rpi"
)

const (
	XOsc              = 32e6
	FreqStep          = XOsc / (1 << 19)
	InterruptInterval = 100 * time.Millisecond
)

type SX1276 struct {
	port spi.PortCloser
	spi  spi.Conn

	reset gpio.PinIO
	dio0  gpio.PinIO
	dio1  gpio.PinIO
	dio2  gpio.PinIO
	dio3  gpio.PinIO
	dio4  gpio.PinIO
	dio5  gpio.PinIO
}

func NewSX1276() (sx *SX1276, err error) {
	_, err = host.Init()
	if err != nil {
		return nil, xerrors.Errorf("host.Init: %w", err)
	}

	sx = &SX1276{
		reset: rpi.P1_12,
		dio0:  rpi.P1_29,
		dio1:  rpi.P1_31,
		dio2:  rpi.P1_32,
		dio3:  rpi.P1_36,
		dio4:  rpi.P1_33,
		dio5:  rpi.P1_11,
	}

	sx.port, err = spireg.Open("SPI0.0")
	if err != nil {
		return nil, xerrors.Errorf(`spireg.Open: %w`, err)
	}

	sx.spi, err = sx.port.Connect(physic.MegaHertz, spi.Mode0, 8)
	if err != nil {
		return nil, xerrors.Errorf("sx.port.Connect: %w", err)
	}

	err = sx.reset.Out(gpio.High)
	if err != nil {
		return nil, xerrors.Errorf("sx.reset.Out: %w", err)
	}
	sx.Reset()

	ver := sx.ReadReg(RegLoRaVERSION)
	if ver != 0x12 {
		return nil, xerrors.Errorf("invalid version: %02X != 0x12\n", ver)
	}

	return
}

func (sx *SX1276) Close() {
	if err := sx.port.Close(); err != nil {
		panic(err)
	}
}

func (sx *SX1276) Reset() {
	sx.reset.Out(gpio.Low)
	time.Sleep(100 * time.Microsecond)
	sx.reset.Out(gpio.High)
	time.Sleep(10 * time.Millisecond)
}

func (sx *SX1276) ReadReg(addr byte) byte {
	var read [2]byte
	err := sx.spi.Tx([]byte{0x7F & addr, 0x00}, read[:])
	if err != nil {
		panic(err)
	}

	return read[1]
}

func (sx *SX1276) ReadRegBurst(addr, n byte) []byte {
	write := make([]byte, n+1)
	read := make([]byte, n+1)
	write[0] = 0x7F & addr
	err := sx.spi.Tx(write, read)
	if err != nil {
		panic(err)
	}

	return read[1:]
}

func (sx *SX1276) WriteReg(addr byte, val ...byte) {
	err := sx.spi.Tx(append([]byte{0x80 | addr}, val...), nil)
	if err != nil {
		panic(err)
	}
}

func (sx *SX1276) SetMode(mode byte) {
	curMode := sx.ReadReg(RegLoRaOPMODE)
	sx.WriteReg(RegLoRaOPMODE, curMode&LORA_OPMODE_MASK|mode)
}

func (sx *SX1276) GetFreq() float64 {
	freqBytes := sx.ReadRegBurst(RegLoRaFRFMSB, 3)
	freqU32 := (uint32(freqBytes[0]) << 16) | (uint32(freqBytes[1]) << 8) | uint32(freqBytes[2])
	return FreqStep * float64(freqU32)
}

func (sx *SX1276) SetFreq(freq float64) {
	freqU32 := uint32(freq / FreqStep)
	sx.WriteReg(RegLoRaFRFMSB, byte(freqU32>>16), byte(freqU32>>8), byte(freqU32))
}

func (sx *SX1276) PktRSSI() float64 {
	return -157 + float64(sx.ReadReg(RegLoRaPKTRSSIVALUE))
}

func (sx *SX1276) PktSNR() float64 {
	return float64(int8(sx.ReadReg(RegLoRaPKTSNRVALUE))) / 4.0
}

func (sx *SX1276) PktFEI(bw float64) float64 {
	// Estimated frequency error is stored as a signed 20-bit integer in
	// RegFeiMsb, RegFeiMid, and RegFeiLsb.
	b := sx.ReadRegBurst(RegLoRaFEIMSB, 3)

	// Make a signed 64-bit integer.
	fei := int64(b[0])<<16 | int64(b[1])<<8 | int64(b[2])

	// Sign-extension and multiply by 2^24 (64 - 20 = 44 and 44 - 24 = 20)
	fei <<= 44
	fei >>= 20

	// Divide by Fxtal
	feiF64 := float64(fei) / XOsc

	// Multiply by bandwidth (kHz) / 500
	return feiF64 * bw / 500
}

func (sx *SX1276) StartRX(ctx context.Context) (pkts chan []byte) {
	pkts = make(chan []byte)

	sx.WriteReg(RegLoRaDIOMAPPING1, 0|
		LORA_DIOMAPPING1_DIO0_00,
	)

	err := sx.dio0.In(gpio.PullDown, gpio.RisingEdge)
	if err != nil {
		log.Fatalf("%+v\n", xerrors.Errorf("sx.dio0.In: %w", err))
	}

	go func() {
		for {
			select {
			case <-ctx.Done():
				close(pkts)
				return
			default:
				if sx.dio0.WaitForEdge(InterruptInterval) {
					pkt, err := sx.rx()
					if err != nil {
						log.Printf("%+v\n", err)
						continue
					}
					pkts <- pkt
				}
			}
		}
	}()

	sx.SetMode(LORA_OPMODE_RECEIVER)

	return pkts
}

func (sx *SX1276) rx() (pkt []byte, err error) {
	// Get the current IRQ flags.
	irqFlags := sx.ReadReg(RegLoRaIRQFLAGS)

	sx.WriteReg(RegLoRaIRQFLAGS, LORA_IRQFLAGS_RXDONE_MASK)

	// Check ValidHeader flag.
	if irqFlags&LORA_IRQFLAGS_VALIDHEADER == 0 {
		sx.WriteReg(RegLoRaIRQFLAGS, LORA_IRQFLAGS_VALIDHEADER)
		return nil, errors.New("invalid header")
	}

	// Check the payload had a CRC.
	if sx.ReadReg(RegLoRaHOPCHANNEL)&LORA_HOPCHANNEL_CRCONPAYLOAD_ON == 0 {
		return nil, errors.New("missing crc")
	}

	// Check the CRC was good.
	if irqFlags&LORA_IRQFLAGS_PAYLOADCRCERROR != 0 {
		sx.WriteReg(RegLoRaIRQFLAGS, LORA_IRQFLAGS_PAYLOADCRCERROR)
		return nil, errors.New("invalid crc")
	}

	rxBytes := sx.ReadReg(RegLoRaRXNBBYTES)          // Get the number of received bytes.
	rxAddr := sx.ReadReg(RegLoRaFIFORXCURRENTADDR)   // Get the current Rx FIFO starting address.
	sx.WriteReg(RegLoRaFIFOADDRPTR, rxAddr)          // Move the FIFO to the beginning of the received bytes.
	payload := sx.ReadRegBurst(RegLoRaFIFO, rxBytes) // Read the payload from the FIFO.

	return payload, nil
}

func init() {
	log.SetFlags(log.Lshortfile | log.Lmicroseconds)
}
