"""Run the actual ArduinoJson registration payload/parser on Windows, without a board or HTTP.

Uses the locally installed Android NDK host compiler (no Android target).
ARDUINOJSON_DIR can override the user's Arduino/libraries/ArduinoJson/src path.
"""
import _ctypes
import ctypes
import os
from pathlib import Path
import subprocess
import tempfile

here = Path(__file__).resolve().parent
root = here.parents[1]
ndk = sorted((Path(os.environ['LOCALAPPDATA']) / 'Android/Sdk/ndk').iterdir())[-1]
tools = ndk / 'toolchains/llvm/prebuilt/windows-x86_64/bin'
json = Path(os.environ.get('ARDUINOJSON_DIR', str(Path.home() / 'Documents/Arduino/libraries/ArduinoJson/src')))
with tempfile.TemporaryDirectory(prefix='green2-registration-') as tmp:
    build = Path(tmp)
    macros = [f'-DARDUINOJSON_ENABLE_{x}=0' for x in
              ('STD_STRING', 'STD_STREAM', 'STRING_VIEW', 'ARDUINO_STREAM', 'ARDUINO_PRINT', 'ARDUINO_STRING', 'PROGMEM')]
    dirs = [here / 'crt', root / 'projectsard/green2_testprosh', json]
    obj = build / 'host.obj'
    subprocess.run([str(tools / 'clang++.exe'), '--target=x86_64-w64-windows-gnu',
                    '-std=c++17', '-O2', '-Wall', '-Werror', '-ffreestanding', '-fno-builtin',
                    '-fno-stack-protector', '-mno-stack-arg-probe', '-nostdinc++',
                    '-fno-exceptions', '-fno-rtti', '-fno-use-cxa-atexit',
                    *macros, *['-I' + str(p) for p in dirs], '-c', str(here / 'host.cpp'), '-o', str(obj)], check=True)
    exports = build / 'msvcrt.def'
    symbols = 'malloc realloc free abort strtod memcpy memmove memset memcmp memchr strlen strcmp strncmp'
    exports.write_text('LIBRARY msvcrt.dll\nEXPORTS\n' + '\n'.join(symbols.split()) + '\n')
    lib = build / 'msvcrt.lib'
    subprocess.run([str(tools / 'llvm-dlltool.exe'), '-m', 'i386:x86-64', '-d', str(exports), '-l', str(lib)], check=True)
    dll = build / 'test.dll'
    subprocess.run([str(tools / 'ld.lld.exe'), '-flavor', 'link', '/dll', '/noentry', '/machine:x64',
                    '/export:registrationTests', '/nodefaultlib', str(obj), str(lib), '/out:' + str(dll)], check=True)
    loaded = ctypes.CDLL(str(dll))
    line = loaded.registrationTests()
    _ctypes.FreeLibrary(loaded._handle)
    if line:
        raise SystemExit(f'Registration test failed at host.cpp:{line}')
print('Green2 console framing, factory payload and post-write readback checks passed; no HTTP requests made.')
