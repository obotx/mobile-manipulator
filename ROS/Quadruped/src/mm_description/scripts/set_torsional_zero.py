#!/usr/bin/env python3
import sys
import xml.etree.ElementTree as ET

def main():
    urdf_str = sys.stdin.read()
    root = ET.fromstring(urdf_str)
    
    count = 0
    for collision in root.iter('collision'):
        surface = collision.find('surface')
        if surface is None:
            surface = ET.SubElement(collision, 'surface')
        
        friction = surface.find('friction')
        if friction is None:
            friction = ET.SubElement(surface, 'friction')
        
        torsional = friction.find('torsional')
        if torsional is None:
            torsional = ET.SubElement(friction, 'torsional')
        
        coeff = torsional.find('coefficient')
        if coeff is None:
            coeff = ET.SubElement(torsional, 'coefficient')
        coeff.text = '0.0'
        count += 1
        
    # Preserve formatting
    ET.indent(root, space='  ')
    print(ET.tostring(root, encoding='unicode'))

if __name__ == '__main__':
    main()