#!/bin/bash

mkdir -p ROOT/tmp/m1OASYS_X2/
cp "../domelist m1OASYS.txt" ROOT/tmp/m1OASYS_X2/
cp "../build/Release/libm1OASYS.dylib" ROOT/tmp/m1OASYS_X2/

if [ ! -z "$installer_signature" ]; then
# signed package using env variable installer_signature
pkgbuild --root ROOT --identifier org.rti-zone.m1OASYS_X2 --sign "$installer_signature" --scripts Scritps --version 1.0 m1OASYS_X2.pkg
pkgutil --check-signature ./m1OASYS_X2.pkg
else
pkgbuild --root ROOT --identifier org.rti-zone.m1OASYS_X2 --scripts Scritps --version 1.0 m1OASYS_X2.pkg
fi

rm -rf ROOT
