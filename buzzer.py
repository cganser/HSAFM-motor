#!/home/afm/python/daq_venv/bin/python

#Passive buzzer control
#
#Manual list of frequencies and times
#freq = [1000,0,1000,0,500,0,0,0,0,0,0]
#time = [0.1,0.001,0.1,0.001,0.1,0.01,0.2,0.01,0.1,0.01,0.1]
#buzzer.setMelody(freq,time)


#Note input with same duration and pause length
#time = 0.1
#pause = 0.05

#buzzer.setMelody2("C5 C5 C5 C5 Ab4 C5 Bb4 C5",time,pause)
#buzzer.playMelody()



import RPi.GPIO as GPIO

from time import sleep

class ApproachBuzzer():
    def __init__(self,pin=11,buzzer="passive"):
        self.pin = pin

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin,GPIO.OUT)

        if buzzer == "passive":
            self.buzzerType = 0
            self.freq = 100
            self.dc = 50

            self.fList = []
            self.tList = []

            finished = 0

            while finished == 0:
                try:
                    self.pwm = GPIO.PWM(self.pin,self.freq)
                    finished = 1
                except RuntimeError:
                    sleep(0.1)
                    pass

            self.pwm.start(0)
        else:
            self.buzzerType = 1


    def noteToFreq(self,note,octave):
        f0 = 440

        if note == 'C':
            n=-9
        elif note == 'Db':
            n=-8
        elif note == 'D':
            n=-7
        elif note == 'Eb':
            n=-6
        elif note == 'E':
            n=-5
        elif note == 'F':
            n=-4
        elif note == 'Gb':
            n=-3
        elif note == 'G':
            n=-2
        elif note == 'Ab':
            n=-1
        elif note == 'A':
            n = 0
        elif note == 'Bb':
            n = 1
        elif note == 'B':
            n = 2
        else:
            n = 0

        freq = f0*2**(n/12-4+octave)
        return freq

    def playSound(self,freq,time):
        if self.buzzerType == 0:
            self.pwm.ChangeFrequency(freq)
            self.pwm.ChangeDutyCycle(self.dc)
            sleep(time)
            self.pwm.start(0)
        else:
            GPIO.output(self.pin, GPIO.HIGH)
            sleep(time)
            GPIO.output(self.pin, GPIO.LOW)


    def setMelody(self,freq_list,time_list):
        self.fList = freq_list
        self.tList = time_list

    def setMelody2(self,notes,gap,bpm, octave_shift=0):
        #the notes should be put in a string like "A2 Bb D4"
        #the time is the length of each note
        #the pause is the silent period between two notes
        self.fList = []
        self.tList = []

        dt = 60.0/bpm

        sep_notes = notes.split(' ')

        for item in sep_notes:
            if item[0] == 'p':
                freq = 0
                note_len = float(item[len(item)-2])/float(item[len(item)-1])
                time = note_len*dt

            else:
                note = item[0:len(item)-3]
                octave = int(item[len(item)-3]) + octave_shift
                freq = self.noteToFreq(note,octave)
                note_len = float(item[len(item)-2])/float(item[len(item)-1])
                time = note_len*dt


            self.fList.append(freq)
            self.tList.append(time)
            self.fList.append(0)
            self.tList.append(gap)

    def playMelody(self):
        for i in range(0,len(self.fList)):
            if self.fList[i] != 0:
                self.playSound(self.fList[i], self.tList[i])
            else:
                sleep(self.tList[i])

    def playStandardSound(self,nr):

        if self.buzzerType == 0:
            if nr == 1:
                self.setMelody2("A414 A414 B424",0.05,120)
            if nr == 2:
                self.setMelody2("D514 D514 D514 D534 Bb424 C524 D524 C514 D524",0.05,180)
            if nr == 3:
                self.setMelody2("A424",0.05,120)
            if nr == 4:
                self.setMelody2("G414 G414 G414 Eb434",0.05,120)

            self.playMelody()

        else:
            self.playSound(0,2)

    def sweep(self,df,dt,startN,N):
        for i in range(startN,N):
            f = i*df
            self.playSound(f,dt)


#buz = ApproachBuzzer(buzzer="passive")
#buz.setMelody2("C514 C514 C514 C524 Ab414 Bb524 C524 Bb514 C524",0.01,90)
#buz.playMelody()
#buz.sweep(100,0.1,1,7000)
#buz.playSound(2800,10)
