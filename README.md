# XYZ_Ctrl_L206 — Control XY Lab 206

Repositorio del controlador de etapa XY (Lab 206).

## Estructura

```text
XYZ_Control_Lab206/
├── MycoViT_XY_Controller/   ← firmware actual (STM32 NUCLEO-F767ZI)
├── XYZ_Ctrl_L206_v0.1/      ← legacy Arduino (solo referencia)
├── Docs/                    ← (si aplica a nivel raíz)
├── README.md
└── .gitignore
```

| Carpeta | Rol |
|---------|-----|
| **`MycoViT_XY_Controller/`** | **Proyecto activo:** CubeMX + Makefile, lazo RT @ 1 MHz, C(z)/LUT micrométrico, protocolo serial 1 Mbps |
| **`XYZ_Ctrl_L206_v0.1/`** | Sketch Arduino histórico (L298N / Mega). No desarrollar aquí |

## Firmware activo (STM32)

- Placa: **NUCLEO-F767ZI**
- Build / flash: ver `MycoViT_XY_Controller/Docs/20260709_1039_Guia_Build_Flash_Serial.md`
- Plan micrométrico: `MycoViT_XY_Controller/Docs/20260714_0032_Plan_Implementacion_Control_Micrometrico_Rapido.md`
- Avance día 2026-07-14: `MycoViT_XY_Controller/Docs/20260714_2354_Reporte_Avance_Dia_Control_Micrometrico.md`

Host GUI (Python): repositorio aparte → [XYZ_Ctrl_L206_GUI](https://github.com/Nahzap/XYZ_Ctrl_L206_GUI).

## Remote

```text
origin → https://github.com/Nahzap/XYZ_Ctrl_L206.git
```

## Build rápido (firmware)

```powershell
cd MycoViT_XY_Controller
# Preparar PATH xPack (ver guía) y luego:
make -j8 all
```
