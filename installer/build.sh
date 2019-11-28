#!/bin/bash

mkdir -p ROOT/tmp/m1OASYS_X2/
cp "../domelist m1OASYS.txt" ROOT/tmp/m1OASYS_X2/
cp "../build/Release/libm1OASYS.dylib" ROOT/tmp/m1OASYS_X2/

PACKAGE_NAME="m1OASYS_X2.pkg"
BUNDLE_NAME="org.rti-zone.m1OASYSX2"

if [ ! -z "$installer_signature" ]; then
	# signed package using env variable installer_signature
	pkgbuild --root ROOT --identifier $BUNDLE_NAME --sign "$installer_signature" --scripts Scripts --version 1.0 $PACKAGE_NAME
	pkgutil --check-signature ./${PACKAGE_NAME}
else
	pkgbuild --root ROOT --identifier $BUNDLE_NAME --scripts Scripts --version 1.0 DomePro_X2.pkg
fi

rm -rf ROOT
