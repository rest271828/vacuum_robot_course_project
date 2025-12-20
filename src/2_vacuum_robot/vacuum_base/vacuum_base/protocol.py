from __future__ import annotations
from dataclasses import dataclass
import struct


@dataclass
class WheelFeedback:
    """MCU -> host feedback."""
    v_l_mm_s: float
    v_r_mm_s: float
    yaw_deg: float


class VacuumSerialProtocol:
    # -------- TX (Host -> MCU) --------
    # 0xAA 0x00 + float32(v_l_cmd) + float32(v_r_cmd) + 0x00 0x55
    TX_HEAD1 = 0xAA
    TX_HEAD2 = 0x00
    TX_TAIL1 = 0x00
    TX_TAIL2 = 0x55
    TX_LEN = 2 + 4 + 4 + 2  # 12 bytes

    # -------- RX (MCU -> Host) --------
    # 0x55 0x00 + float32(v_l_mm_s) + float32(v_r_mm_s) + float32(yaw_deg) + 0x00 0xAA
    RX_HEAD1 = 0x55
    RX_HEAD2 = 0x00
    RX_TAIL1 = 0x00
    RX_TAIL2 = 0xAA
    RX_LEN = 2 + 12 + 2  # 16 bytes

    @staticmethod
    def pack_tx_cmd(v_l_cmd: float, v_r_cmd: float) -> bytes:
        """
        Pack wheel commands for MCU.
        NOTE: Uses float32 for each wheel.
        Units: whatever MCU expects (you will send mm/s here by default in node).
        """
        return bytes([VacuumSerialProtocol.TX_HEAD1, VacuumSerialProtocol.TX_HEAD2]) + struct.pack(
            "<ff", float(v_l_cmd), float(v_r_cmd)
        ) + bytes([VacuumSerialProtocol.TX_TAIL1, VacuumSerialProtocol.TX_TAIL2])

    @staticmethod
    def parse_rx_frames(rx: bytearray) -> list[WheelFeedback]:
        """
        Parse as many RX frames as possible from buffer.
        Keeps remaining bytes in rx.
        """
        out: list[WheelFeedback] = []

        # Search frame head 0x55 0x00
        i = 0
        while i <= len(rx) - VacuumSerialProtocol.RX_LEN:
            if rx[i] != VacuumSerialProtocol.RX_HEAD1 or rx[i + 1] != VacuumSerialProtocol.RX_HEAD2:
                i += 1
                continue

            end = i + VacuumSerialProtocol.RX_LEN
            if rx[end - 2] != VacuumSerialProtocol.RX_TAIL1 or rx[end - 1] != VacuumSerialProtocol.RX_TAIL2:
                # bad tail, resync by skipping one byte
                i += 1
                continue

            payload = rx[i + 2 : i + 14]  # 12 bytes
            v_l_mm_s, v_r_mm_s, yaw_deg = struct.unpack("<fff", payload)
            out.append(WheelFeedback(v_l_mm_s=v_l_mm_s, v_r_mm_s=v_r_mm_s, yaw_deg=yaw_deg))

            # consume
            del rx[i:end]

        # avoid runaway buffer
        if len(rx) > 4096:
            del rx[:-64]

        return out
