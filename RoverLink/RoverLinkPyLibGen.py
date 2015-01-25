#!/usr/bin/env python2.6

# This is the C source file header generator for the BYU Mars Rover comms protocol.
# As with all else, it is inspired by the MavLink project, except that am simplifining the generator.
# Honestly, it is because I have no idea how to imitate the MavLink generator, and this is fun.
# -Steve



import xml.etree.ElementTree as ET
import datetime

def printListing(root,string,level=0):
	string += '  '*level + root.tag + ' ' + str(root.get("name")) + "\n"
	for element in root.findall("*"): # Can also do './'  root.iter('*'): #
		string = printListing(element,string,level+1)
	return string


fileName = "RoverLink.xml"
headerFileName = "RoverLinkDefs.py"

tree = ET.parse(fileName)
root = tree.getroot()

#print ET.tostringing(root) # Echos the entire xml file perfectly
#print ET.tostringinglist(root) # Same as above, but in a single line
output = ''
# print printListing(root,output)



output = '''# BYU Mars Rover Communications Protocol Definitions
# Use tabs=4, written in Notepad++
#
# !!! BE ADVISED !!!
# This file is generated from RoverLink.xml using RoverLinkGenerator.py
# If this file needs to be modified, make the appropriate changes in the XML file.
# The XML file is used to generate the same protocol in the Basestation GUI, 
# thus synchronizing the entire comms system.
#
# Protocol notes are inserted at the bottom of this document.
'''

output += '\n# Build Timestemp: '+str(datetime.datetime.utcnow().isoformat())

output += '\n\nclass RoverLinkDefs:\n\n'


output += "\t# Constants\n"
for entry in root.findall("./messages/message"):
	output += '\tROVERLINK_'+str(entry.get("name")).upper()+'_MSG_ID = '+str(entry.get("id"))+'\n'
output += "\n"

for entry in root.findall("./messages/message"):
	output += '\tROVERLINK_'+str(entry.get("name")).upper()+'_MSG_LEN = '
	byteCount = 0;
	for value in entry.iter("field"):
		if "int8_t" in str(value.get("type")): byteCount += 1
		elif "int16_t" in str(value.get("type")): byteCount += 2
		elif "int32_t" in str(value.get("type")): byteCount += 4
		elif "int64_t" in str(value.get("type")): byteCount += 8
		elif "systemState_t" in str(value.get("type")): byteCount += 1
		elif "messageMask_t" in str(value.get("type")): byteCount += 2
		elif "broadcast_t" in str(value.get("type")): byteCount += 1
	output += str(byteCount)+'\n'
output += '\n\n'

	
# output += "\tROVERLINK_MSG_LEN_ARRAY[] = [\n"
# for entry in root.findall("./messages/message"):
	# output += '\t\tROVERLINK_'+str(entry.get("name")).upper()+'_MSG_LEN,\n'
# output += '\t\t]\n'

output += "\tROVERLINK_MSG_LEN_DICT = {\n"
for entry in root.findall("./messages/message"):
	output += '\t\tROVERLINK_'+str(entry.get("name")).upper()+'_MSG_LEN : -1,\n' # Finish this!
output += '\t\t}\n\n'

for entry in root.findall("./constants/constant"):
	output += '\tROVERLINK_'+entry.get("name")+' = '+entry.get("value")+';\n'
	
output += "\n\n"
for entry in root.findall("./enums/enum"):
	output += "\t# "+entry.find("description").text+"\n"
	# output += '\t# '+entry.get("name")+'\n'
	for value in entry.iter("entry"):
		output += '\t'+str(value.get("name"))+' = '+str(value.get("value"))+'\n'
	output += '\n'

output += "# -- Message Class Instances\n"
for entry in root.findall("./messages/message"):
	output += "# "+entry.find("description").text+"\n"
	output += 'class '+str(entry.get("name")).lower()+':\n'
	output += '\tdef __init__(self,):\n'
	for value in entry.iter("field"):
		output += '\t\tself.'+str(value.get("name"))+' = '+str(value.get("initValue",0)) #+'\n'
		output += "\t# "+value.text+"\n"
	output += '\tdef toList():\n'
	output += '\t\treturn self.__dict__\n'
	output += '\n\n'



file = open(headerFileName,'w')

file.write(output)
file.close()


#print output