# OdinGenerated
# ODIN_SCRIPT_NATURE = GRAPH

# C3D export script
# last modifications: 
#     - 02/09/2016 include residual information (occlusion flag)
#     - 2024-07-25 (Omar Galarraga): Adding joint kinematics and kinetics data. Change FilterData option. Include Comment and Condition in filename 
#     - 2024-07-29 (Omar Galarraga) : Adding Gait Event Data
#     - 2024-08-14 (Omar Galarraga): Adding metadata (trial, subject, manufacturer, gait scores)

from numpy import *
import btk
import os
from fractions import gcd
from OdinPy import *
from copy import deepcopy
import re

inputs = {
# Marker position
"MFilter" : {"locatortype":"tsgroup", "groupname":"MFilter", "calclevel":"Calc"}, 
"RefPointFilter" : {"locatortype":"tsgroup", "groupname":"RefPointFilter", "calclevel":"Calc"}, 
"Marker" : {"locatortype":"tsgroup", "groupname":"Marker", "calclevel":"Acq"},
"RefPoint" : {"locatortype":"tsgroup", "groupname":"RefPoint", "calclevel":"Calc"}, 

#Analog
"Analog" : {"locatortype":"tsgroup", "groupname":"Analog", "calclevel":"Acq"},

#LCS
"evb" : {"locatortype":"tsgroup", "groupname":"RefEVB", "calclevel":"Calc"},

# Angles
"EA" : {"locatortype":"tsgroup", "groupname":"JointAngle", "calclevel":"Calc"},
"EAFilter" : {"locatortype":"tsgroup", "groupname":"JointAngleFilter", "calclevel":"Calc"},

# Force
"Force" : {"locatortype":"tsgroup", "groupname":"Force", "calclevel":"Calc"},
"Point" : {"locatortype":"tsgroup", "groupname":"ForcePOA", "calclevel":"Calc"}, 
"fpinfo": {"path":"Acq/ForcePlates","locatortype":"xmove"},

# Moment
"Moment" : {"locatortype":"tsgroup", "groupname":"JointMoment", "calclevel":"Calc"}, 
"MomentFilter" : {"locatortype":"tsgroup", "groupname":"JointMomentFilter", "calclevel":"Calc"}, 
 
# Power
"Power" : {"locatortype":"tsgroup", "groupname":"JointPower", "calclevel":"Calc"}, 
"PowerFilter" : {"locatortype":"tsgroup", "groupname":"JointPowerFilter", "calclevel":"Calc"},

# Event
"Left" : {"groupname":"Left", "locatortype":"eventgroup", "calclevel":"UserInput"},
"Right" : {"groupname":"Right", "locatortype":"eventgroup", "calclevel":"UserInput"},

# Trial Information
"Name": {"path":"UserInput/Subject/ID","locatortype":"xmove"},
"RepLabel": {"path":"UserInput/RepLabel","locatortype":"xmove"},
"Date" : { "path":"Acq/Date","locatortype":"xmove"},
"Condition": {"path":"UserInput/Trial/Condition","locatortype":"xmove"},
"Comment": {"path":"UserInput/Trial/Comment","locatortype":"xmove"},
"Fields": {"path":"UserInput","locatortype":"xmove"},
"ManufacturerInfo": {"path":"Acq/MeasurementSystem","locatortype":"xmove"},
}

outputs = {
"Value" : { "locatortype": "xmove", "path": "UserInput/Trial/C3D_Export"},
}


def compute(Marker, RefPoint, MFilter, RefPointFilter, EA, EAFilter, Analog, evb, Force, Moment, MomentFilter, Power, PowerFilter, Point, fpinfo, Left, Right, Name, RepLabel, Date, Comment, Condition, Fields, ManufacturerInfo, FilterData = True):

    processing = {'Marker_Filtered': 0, 'VirtualMarker_Filtered': 0, 'Angle_Filtered': 0, 'Moment_Filtered':0, 'Power_Filtered': 0}
    
    # choosing the option of exporting filter data instead of raw data
    if FilterData:
        Marker = deepcopy(MFilter)
        processing['Marker_Filtered'] = 1        
        RefPoint = deepcopy(RefPointFilter)
        processing['VirtualMarker_Filtered'] = 1        
        if EAFilter is not None:
            EA = deepcopy(EAFilter)
            processing['Angle_Filtered'] = 1
        if MomentFilter is not None:
            Moment = deepcopy(MomentFilter)
            processing['Moment_Filtered'] = 1
        if PowerFilter is not None:
            Power = deepcopy(PowerFilter)
            processing['Power_Filtered'] = 1
            
    Rates = array([Marker[0]['Rate']])
    if Analog != None:
        for i in Analog:
            Rates = append(Rates,  i['Rate'])
    CommonRate = reduce(gcd, Rates)

    MRate = Marker[0]['Rate']
    MarkerRatio = MRate/CommonRate

    NbFrame = Marker[0]['Data']['value'][0:-1:MarkerRatio, :].shape[0]

    acq = btk.btkAcquisition()
    acq.Init(0,NbFrame)
    acq.SetPointFrequency(CommonRate)
    acq.Update()

    # Adding Real Markers
    for m in Marker:
        newpoint = btk.btkPoint(m['Channel'], acq.GetPointFrameNumber()) 
        newpoint.SetValues(m['Data']['value'][0:-1:MarkerRatio,:])
        NbFrames = m['Data']['value'][0:-1:MarkerRatio,:].shape[0]
        residuals = -m['Data']['occluded'][0:-1:MarkerRatio].reshape([NbFrames,1]).astype(float)
        newpoint.SetResiduals(residuals)
        acq.AppendPoint(newpoint)
        acq.Update()

    # Adding Virtual Markers
    for rf in RefPoint:
        newpoint = btk.btkPoint('V.' + rf['Channel'], acq.GetPointFrameNumber()) 
        newpoint.SetValues(rf['Data']['value'][0:-1:MarkerRatio, :])
        NbFrames = rf['Data']['value'][0:-1:MarkerRatio,:].shape[0]
        residuals = -rf['Data']['occluded'][0:-1:MarkerRatio].reshape([NbFrames,1]).astype(float)
        newpoint.SetResiduals(residuals)
        acq.AppendPoint(newpoint)
        acq.Update()

    # Adding Force plate metadata
    if fpinfo != None:
        acq = ForcePlateType1(fpinfo,  Force,  Point,  acq, CommonRate)
    
    #adding analog channels
    for a in Analog:
        RateRatio = a['Rate']/CommonRate
        newAnalog=btk.btkAnalog(a['Channel'],  len(a['Data']['value'][0:-1:RateRatio]))
        newAnalog.SetValues(a['Data']['value'][0:-1:RateRatio])
        acq.AppendAnalog(newAnalog)
        acq.Update()

    #Adding JointAngles
    if EA is not None:
        for angle in EA:
            newpoint = btk.btkPoint(angle['Channel'], acq.GetPointFrameNumber())
            newpoint.SetValues(angle['Data']['value'][0:-1:MarkerRatio,:])
            NbFrames = angle['Data']['value'][0:-1:MarkerRatio,:].shape[0]
            residuals = -angle['Data']['occluded'][0:-1:MarkerRatio].reshape([NbFrames,1]).astype(float)
            newpoint.SetResiduals(residuals)
            acq.AppendPoint(newpoint)
            acq.Update()
            
    #Adding LCS
    if evb is not None:
        for lcs in evb:
            newpoint = btk.btkPoint(lcs['Channel']+".LCS", acq.GetPointFrameNumber())
            newpoint.SetValues(lcs['Data']['value'][0:-1:MarkerRatio,:])
            NbFrames = lcs['Data']['value'][0:-1:MarkerRatio,:].shape[0]
            residuals = -lcs['Data']['occluded'][0:-1:MarkerRatio].reshape([NbFrames,1]).astype(float)
            newpoint.SetResiduals(residuals)
            acq.AppendPoint(newpoint)
            acq.Update()            
    
    #Adding JointMoments
    if Moment is not None:
        for mom in Moment:
            newpoint = btk.btkPoint(mom['Channel']+'Moment', acq.GetPointFrameNumber())
            newpoint.SetValues(mom['Data']['value'][0:-1:MarkerRatio,:])
            NbFrames = mom['Data']['value'][0:-1:MarkerRatio,:].shape[0]
            residuals = -mom['Data']['occluded'][0:-1:MarkerRatio].reshape([NbFrames,1]).astype(float)
            newpoint.SetResiduals(residuals)
            acq.AppendPoint(newpoint)
            acq.Update()
            
    #Adding JointPowers
    if Power is not None:
        for powj in Power:
            newpoint = btk.btkPoint(powj['Channel']+'Power', acq.GetPointFrameNumber())
            newpoint.SetValues(powj['Data']['value'][0:-1:MarkerRatio,:])
            NbFrames = powj['Data']['value'][0:-1:MarkerRatio,:].shape[0]
            residuals = -powj['Data']['occluded'][0:-1:MarkerRatio].reshape([NbFrames,1]).astype(float)
            newpoint.SetResiduals(residuals)
            acq.AppendPoint(newpoint)
            acq.Update()
    
    #Adding Events
    #iev = 0
    for event in Left['data']:
        #msg(msg=event)
        if event[0] == 2:
            eventdesc = "Foot Off Event"
        else:
            eventdesc = "Foot Strike Event"
        newEvent = btk.btkEvent(Left['eventname'][event[0]], round(event[1]*MRate), 'Left', btk.btkEvent.Manual, Name, eventdesc, 0)
        acq.AppendEvent(newEvent)

    for event in Right['data']:
        #msg(msg=event)
        if event[0] == 2:
            eventdesc = "Foot Off Event"
        else:
            eventdesc = "Foot Strike Event"
        newEvent = btk.btkEvent(Right['eventname'][event[0]], round(event[1]*MRate), 'Right', btk.btkEvent.Manual, Name, eventdesc, 0)
        acq.AppendEvent(newEvent)        
        
    ###Adding Metadata
    metadata = acq.GetMetaData()
    
    #SUBJECT INFO
    subjects = btk.btkMetaData('SUBJECTS')
    subjects.AppendChild(btk.btkMetaData('NAMES', Name))
    if "Static" in Comment or "static" in Comment:
        is_static = 1
    else:
        is_static = 0
    subjects.AppendChild(btk.btkMetaData('IS_STATIC', is_static))
    subjects.AppendChild(btk.btkMetaData('PROTOCOL', Fields['Protocol']['Acquired']))
    metadata.AppendChild(subjects)
    acq.Update()
    
    subject_info = btk.btkMetaData(Name.replace(' ', '_').upper())
    excluded_fields = ['Experimentator', 'ID']
    for sinfo in Fields['Subject']:
        if sinfo not in excluded_fields:
            if sinfo == "Weight":
                subject_info.AppendChild(btk.btkMetaData('Bodymass', Fields['Subject'][sinfo]))
            elif sinfo == "Height":
                subject_info.AppendChild(btk.btkMetaData(sinfo, Fields['Subject'][sinfo]*1000))                
            elif sinfo == "DateOfBirth":
                month = str(Fields['Subject'][sinfo]['Month'])
                day = str(Fields['Subject'][sinfo]['Month'])
                if len(month) == 1:
                    month = "0"+month
                if len(day) == 1:
                    day = "0"+day
                subject_info.AppendChild(btk.btkMetaData('Birthdate', str(Fields['Subject'][sinfo]['Year'])+'-'+month+'-'+day))
            else:
                subject_info.AppendChild(btk.btkMetaData(sinfo, Fields['Subject'][sinfo]))
    metadata.AppendChild(subject_info)
    acq.Update()
    
    #TRIAL INFO
    trial = btk.btkMetaData('TRIAL')
    mes = str(Date['Month'])
    if len(mes) == 1:
        mes = "0"+mes
    dia = str(Date['Day'])        
    if len(dia) == 1:
        dia = "0"+dia        
    trial.AppendChild(btk.btkMetaData('DATE', str(Date['Year']) + '-' + mes + '-' + dia))
    cond = btk.btkMetaData('CONDITION', Condition)
    comm = btk.btkMetaData('COMMENT', Comment)
    trial.AppendChild(cond)
    trial.AppendChild(comm)
    exp_list = Fields['Subject']['Experimentator'].replace(" ", "").split(",")
    #msg(msg=" ".join(exp_list))
    experimentator = btk.btkMetaData("EXPERIMENTATOR", exp_list[0])
    experimentator.GetInfo().SetDimensions([10, len(exp_list)])
    #msg(msg=len(exp_list))
    if len(exp_list) > 1:
        for exp in range(1,len(exp_list)):
            experimentator.GetInfo().SetValue(exp, exp_list[exp])
            experimentator.Update()
            #msg(msg=experimentator.GetInfo().ToString())
    trial.AppendChild(experimentator)
    #msg(msg=experimentator.GetInfo().ToString())
    
    #MANUFACTURER
    to_include = ['SoftwareName','SoftwareVersion','DataServerName', 'DataServerVersion']
    manufacturer = btk.btkMetaData('MANUFACTURER')
    manufacturer.AppendChild(btk.btkMetaData('COMPANY', ManufacturerInfo['Name']+" "+ManufacturerInfo['Company']))

    for minfo in to_include: 
        manufacturer.AppendChild(btk.btkMetaData(minfo, ManufacturerInfo[minfo]))

    metadata.AppendChild(manufacturer)
    acq.Update()

    #PROCESSING
    processed = btk.btkMetaData('PROCESSING')

    for pinfo in processing: 
        processed.AppendChild(btk.btkMetaData(pinfo, processing[pinfo]))

    processed.AppendChild(btk.btkMetaData('ProtocolModified', Fields['Protocol']['Modified']))
    metadata.AppendChild(processed)
    acq.Update()    
    
    #GAIT PARAMETERS AND SCORES
    analysis = btk.btkMetaData('ANALYSIS')
    #msg(msg=Fields['GaitParameters'])
    for ginfo in Fields['GaitParameters']: 
        if Fields['GaitParameters'][ginfo]:
            analysis.AppendChild(btk.btkMetaData(ginfo, Fields['GaitParameters'][ginfo]))
        else:
            analysis.AppendChild(btk.btkMetaData(ginfo, ''))

    #msg(msg=Fields['GaitIndex'])
    for ginfo in Fields['GaitIndex']:
        msg(msg=ginfo)
        msg(msg=Fields['GaitIndex'][ginfo])
        if  Fields['GaitIndex'][ginfo]:
            if 'MAP' in ginfo:
                #msg(msg=type(ginfo))
                maptemp = Fields['GaitIndex'][ginfo]
                maptemp = re.sub(' +', ' ', maptemp)
                maptemp = re.sub('[\[\]]', '', maptemp)
                maptemp = maptemp.split(" ")[1:]
                maplist = [float(i) for i in maptemp]
                mapgps = btk.btkMetaData(ginfo, maplist)
                mapgps.GetInfo().SetDimensions((1,9))
                #for gvs in range(1,9):
                #    mapgps.GetInfo().SetValue(gvs, float(maplist[gvs]))
                #mapgps.GetInfo().SetFormat(btk.btkMetaDataInfo.Real)
                #msg(msg=mapgps.GetInfo().ToString())
                analysis.AppendChild(mapgps)
            else:
                analysis.AppendChild(btk.btkMetaData(ginfo, Fields['GaitIndex'][ginfo]))
        else:        
            analysis.AppendChild(btk.btkMetaData(ginfo, ''))
        analysis.Update()
            
    metadata.AppendChild(analysis)
    acq.Update()        
    
    metadata.AppendChild(trial)
    acq.Update()

    # Saving the data in the C3D file
    if not os.path.isdir('C:\Codamotion\Odin_x64\Data'): 
        os.makedirs('C:\Codamotion\Odin_x64\Data')

    #FileName =  'C:\\Codamotion\\Odin_x64\\Data\\' + Name +'_' + str(Date['Year']) +'_' + str(Date['Month']) +'_' + str(Date['Day']) + '_' + str(RepLabel) + '.c3d'
    FileName =  'C:\\Codamotion\\Odin_x64\\Data\\' + Name +'_' + str(Date['Year']) +'_' + str(Date['Month']) +'_' + str(Date['Day']) + '_' + Comment + '_' + Condition +'_' + str(RepLabel) + '.c3d'
    
    writer = btk.btkAcquisitionFileWriter() 
    writer.SetInput(acq) 
    writer.SetFilename(FileName) 
    writer.Update()

    return{"Value": None}


def ForcePlateType1(FPInfo, Force, Point, acq, CommonRate):

    NbFP = len(Force)
    
    # Creating forceplate metadata
    new_md = btk.btkMetaData('FORCE_PLATFORM') 
    Used = btk.btkMetaData('USED', int(NbFP)) # should be an integer value
    Type = btk.btkMetaData('TYPE',  [1 for x in range(NbFP)]) # should be an integer value
    Type.GetInfo().SetDimensions([NbFP]) 

    fpcorner = zeros([3,4*NbFP])
    origin = zeros([3,NbFP])
    a = 1
    for fp in FPInfo['ForcePlate']:
        #windll.user32.MessageBoxA( None,  str(fp), "Age not defined", 1)
        corners = fp['Outline']
        fpcorner[:,a*4-4] = [fp['Outline']['Corner'][0]['X'], fp['Outline']['Corner'][0]['Y'], fp['Outline']['Corner'][0]['Z']]
        fpcorner[:,a*4-3] = [fp['Outline']['Corner'][1]['X'], fp['Outline']['Corner'][1]['Y'], fp['Outline']['Corner'][1]['Z']]
        fpcorner[:,a*4-2] = [fp['Outline']['Corner'][2]['X'], fp['Outline']['Corner'][2]['Y'], fp['Outline']['Corner'][2]['Z']]
        fpcorner[:,a*4-1] = [fp['Outline']['Corner'][3]['X'], fp['Outline']['Corner'][3]['Y'], fp['Outline']['Corner'][3]['Z']] 
        if fp['Type'] == 'Kistler':
            origin[:, a-1] = [0.0,  0.0,  0.0]
        else:
            origin[:, a-1] = [fp['CentreOffset']['X'],  fp['CentreOffset']['Y'],  fp['CentreOffset']['Z']]
        a = a+1

    del a

    Corners = btk.btkMetaData('CORNERS',  fpcorner.T.reshape(size(fpcorner))) # should be real value
    Corners.GetInfo().SetDimensions((3,4, NbFP))

    Channels = btk.btkMetaData('CHANNEL',  [x+1 for x in range(6*NbFP)] ) # should be integer value
    Channels.GetInfo().SetDimensions([6, NbFP])

    Origin = btk.btkMetaData('ORIGIN',  origin.T.reshape(size(origin)))  # should be real value
    Origin.GetInfo().SetDimensions([3, NbFP]) 

    new_md.AppendChild(Used) 
    new_md.AppendChild(Type) 
    new_md.AppendChild(Corners) 
    new_md.AppendChild(Channels) 
    new_md.AppendChild(Origin)
    #new_md.AppendChild(Zero)

    acq.GetMetaData().AppendChild(new_md)
    
    
    for i in arange(0, len(Force)):
        # determining orientation
        x = fpcorner[0, 4*i]-fpcorner[0, 4*i+1]
        y = fpcorner[1, 4*i]-fpcorner[1, 4*i+1]
        RateRatio = Force[i]['Rate']/CommonRate
        if x < 0 and y == 0:
            #windll.user32.MessageBoxA( None,  str(0), "Age not defined", 1)
            DimX = mean([fpcorner[0, 4*i+1],  fpcorner[0, 4*i]])
            DimY = mean([fpcorner[1, 4*i+3], fpcorner[1, 4*i]])
            #print('dimX: '+str(DimX))
            #print('dimY: '+str(DimY))
            Fx = -Force[i]['Data']['value'][0:-1:RateRatio,  0]
            Fy = Force[i]['Data']['value'][0:-1:RateRatio,  1]
            Fz = -Force[i]['Data']['value'][0:-1:RateRatio,  2]
            Px = -Point[i]['Data']['value'][0:-1:RateRatio,  0]+DimX
            Py = Point[i]['Data']['value'][0:-1:RateRatio,  1]-DimY
            Pz = Point[i]['Data']['value'][0:-1:RateRatio,  2]
        elif x == 0 and y < 0:
            #windll.user32.MessageBoxA( None,  str(90), "Age not defined", 1)
            DimX = mean([fpcorner[0, 4*i], fpcorner[0, 4*i+3]])
            DimY = mean([fpcorner[1, 4*i+2], fpcorner[1, 4*i+3]])
            Fx = -Force[i]['Data']['value'][0:-1:RateRatio,  1]
            Fy = -Force[i]['Data']['value'][0:-1:RateRatio,  0]
            Fz = -Force[i]['Data']['value'][0:-1:RateRatio,  2]
            Px = -Point[i]['Data']['value'][0:-1:RateRatio,  1]+DimY
            Py = -Point[i]['Data']['value'][0:-1:RateRatio,  0]+DimX
            Pz = Point[i]['Data']['value'][0:-1:RateRatio,  2]
        elif x > 0 and y == 0:
            #windll.user32.MessageBoxA( None,  str(180), "Age not defined", 1)
            DimX = mean([fpcorner[0, 4*i+3], fpcorner[0, 4*i+2]])
            DimY = mean([fpcorner[1, 4*i+1], fpcorner[1, 4*i+2]])
            Fx = Force[i]['Data']['value'][0:-1:RateRatio,  0]
            Fy = -Force[i]['Data']['value'][0:-1:RateRatio,  1]
            Fz = -Force[i]['Data']['value'][0:-1:RateRatio,  2]
            Px = Point[i]['Data']['value'][0:-1:RateRatio,  0]-DimX
            Py = -Point[i]['Data']['value'][0:-1:RateRatio,  1]+DimY
            Pz = Point[i]['Data']['value'][0:-1:RateRatio,  2]
        elif x == 0 and y > 0:
            #windll.user32.MessageBoxA( None,  str(270), "Age not defined", 1)
            DimX = mean([fpcorner[0, 4*i+2], fpcorner[0, 4*i+1]])
            DimY = mean([fpcorner[1, 4*i], fpcorner[1, 4*i+1]])
            Fx = Force[i]['Data']['value'][0:-1:RateRatio,  1]
            Fy = Force[i]['Data']['value'][0:-1:RateRatio,  0]
            Fz = -Force[i]['Data']['value'][0:-1:RateRatio,  2]
            Px = Point[i]['Data']['value'][0:-1:RateRatio,  1]-DimY
            Py = Point[i]['Data']['value'][0:-1:RateRatio,  0]-DimX
            Pz = Point[i]['Data']['value'][0:-1:RateRatio,  2]
        else:
            windll.user32.MessageBoxA( None,  str('The orientation of the plate is not one of the following: 0, 90, 180 and 270 degrees.\nPlease contact Codamotion'), "Age not defined", 1)

        NBFPFrames = len(Fx)

        # Force
        newAnalog=btk.btkAnalog('Fx'+str(i+1) , NBFPFrames)
        newAnalog.SetValues(Fx)
        acq.AppendAnalog(newAnalog)
        acq.Update()
        newAnalog=btk.btkAnalog('Fy'+str(i+1), NBFPFrames)
        newAnalog.SetValues(Fy)
        acq.AppendAnalog(newAnalog)
        acq.Update()
        newAnalog=btk.btkAnalog('Fz'+str(i+1), NBFPFrames)
        newAnalog.SetValues(Fz)
        acq.AppendAnalog(newAnalog)
        acq.Update()
        
        # CoP
        newAnalog=btk.btkAnalog('Px'+str(i+1), NBFPFrames)
        newAnalog.SetValues(Px)
        acq.AppendAnalog(newAnalog)
        acq.Update()
        newAnalog=btk.btkAnalog('Py'+str(i+1), NBFPFrames)
        newAnalog.SetValues(Py)
        acq.AppendAnalog(newAnalog)
        acq.Update()
        newAnalog=btk.btkAnalog('Pz'+str(i+1), NBFPFrames)
        newAnalog.SetValues(Pz)
        acq.AppendAnalog(newAnalog)
        acq.Update()
        
    
    return acq
   