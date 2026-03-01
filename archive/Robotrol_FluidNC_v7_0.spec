# -*- mode: python ; coding: utf-8 -*-


a = Analysis(
    ['Robotrol_FluidNC_v7_0.py'],
    pathex=[],
    binaries=[],
    datas=[('configs', 'configs'), ('model', 'model'), ('data', 'data'), ('Moveo.json', '.'), ('EB15_red.json', '.'), ('EB300.json', '.'), ('gamepad_config.json', '.')],
    hiddenimports=[],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    noarchive=False,
    optimize=0,
)
pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.datas,
    [],
    name='Robotrol_FluidNC_v7_0',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=False,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)
