from pathlib import Path
import sys

Import("env")

sys.path.insert(0, str(Path(env.subst("$PROJECT_DIR")) / "scripts" / "http_hardening"))
from patch_webserver import generate

arch = "esp32" if env.subst("$PIOPLATFORM") == "espressif32" else "esp8266"
package = "framework-arduinoespressif32" if arch == "esp32" else "framework-arduinoespressif8266"
framework = env.PioPlatform().get_package_dir(package)
output = Path(env.subst("$PROJECT_BUILD_DIR")) / "http-libraries" / env.subst("$PIOENV")
library = generate(framework, output, arch)
env.Prepend(LIBSOURCE_DIRS=[str(output)])
# Also prefer the hardened header if another dependency exposes an upstream path.
env.Prepend(CPPPATH=[str(library / "src")])
print("Using project-local hardened HTTP parser: " + str(library))
