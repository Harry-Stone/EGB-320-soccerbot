from tkinter import *
from tkinter import ttk
from time import sleep

form = Tk()
form.title("Soccer Robot GUI")
form.geometry("500x300")

tab_GUI = ttk.Notebook(form)

tab_Play_Soccer = ttk.Frame(tab_GUI)
tab_Drive = ttk.Frame(tab_GUI)
tab_Navigation = ttk.Frame(tab_GUI)
tab_Vision = ttk.Frame(tab_GUI)
tab_Kicker_Dribbler = ttk.Frame(tab_GUI)

tab_GUI.add(tab_Play_Soccer, text="Play Soccer Tests")
tab_GUI.add(tab_Drive, text="Drive Tests")
tab_GUI.add(tab_Navigation, text="Navigation Tests")
tab_GUI.add(tab_Kicker_Dribbler, text="Kicker Dribbler Tests")
tab_GUI.add(tab_Vision, text="Vision Tests")



# === WIDGETS FOR TAB ONE
def Start():
    Label_Running = Label(tab_Play_Soccer, text="Started")
    Label_Running.grid(row=0, column=0, padx=5, pady=5)

def Stop():
    Label_Running = Label(tab_Play_Soccer, text="Stopped")
    Label_Running.grid(row=0, column=0, padx=5, pady=5)

Label_Running = Label(tab_Play_Soccer, text="Not Running")
Label_Running.grid(row=0, column=0, padx=5, pady=5)

Button_Start = Button(tab_Play_Soccer, text="Start", command = Start)
Button_Start.grid(row=1, column=0, padx=5, pady=5)

Button_Stop = Button(tab_Play_Soccer, text="Stop", command = Stop)
Button_Stop.grid(row=2, column=0, padx=5, pady=5)



Goal_Label1 = "No Goal"
def Goal():
    if goal1.get() == 1:
        Goal_Label1 = "Yellow Goal"
    elif goal1.get() == 2:
        Goal_Label1 = "Blue Goal"
    else:
        Goal_Label1 = "No Goal"
        
    Label_Goal = Label(tab_Play_Soccer, text=Goal_Label1)
    Label_Goal.grid(row=0, column=1, padx=5, pady=5)

goal1 = IntVar()


Label_Goal = Label(tab_Play_Soccer, text="Pick a Goal")
Label_Goal.grid(row=0, column=1, padx=5, pady=5)

Radio_Button_No_Goal_Play = Radiobutton(tab_Play_Soccer, text=" No Goal", variable = goal1, value = 0, command = Goal)
Radio_Button_No_Goal_Play.grid(row=1, column=1, padx=5, pady=5)
Radio_Button_Yellow_Goal_Play = Radiobutton(tab_Play_Soccer, text="Yellow Goal", variable = goal1, value = 1, command = Goal)
Radio_Button_Yellow_Goal_Play.grid(row=2, column=1, padx=5, pady=5)
Radio_Button_Blue_Goal_Play = Radiobutton(tab_Play_Soccer, text="Blue Goal", variable = goal1, value = 2, command = Goal)
Radio_Button_Blue_Goal_Play.grid(row=3, column=1, padx=5, pady=5)




def Obstacules():
    Label_Obstacules = Label(tab_Play_Soccer, text=Slider_Obstacules1.get())
    Label_Obstacules.grid(row=1, column=2, padx=5, pady=5)


Label_Obstacules = Label(tab_Play_Soccer, text="Number of Obstacules")
Label_Obstacules.grid(row=0, column=2, padx=5, pady=5)

Label_Obstacules = Label(tab_Play_Soccer, text="0")
Label_Obstacules.grid(row=1, column=2, padx=5, pady=5)

Button_Obstacule = Button(tab_Play_Soccer, text="Obstacules", command = Obstacules)
Button_Obstacule.grid(row=2, column=2, padx=5, pady=5)

Slider_Obstacules1 = Scale(tab_Play_Soccer, from_ = 3, to = 0, resolution = 1)
Slider_Obstacules1.grid(row = 1, column = 3)



def goodbye():
    print("clean")
    
Button_Clean = Button(tab_Play_Soccer, text = 'QUIT', command = goodbye)
Button_Clean.grid(row = 4, column = 2, padx=5, pady=5)


# === WIDGETS FOR TAB TWO

def Top_Speed():
    Label_Speed = Label(tab_Drive,text = '     Top Speed     ')
    Label_Speed.grid(row = 0,column = 1, padx = 0)

def Slow_Speed():
    Label_Speed = Label(tab_Drive,text = '      Slow Speed     ')
    Label_Speed.grid(row = 0,column = 1, padx = 0)

def Forward_Angle_Speed():
    Label_Speed = Label(tab_Drive,text = '    Angle Speed    ')
    Label_Speed.grid(row = 0,column = 1, padx = 0)


Label_Speed = Label(tab_Drive,text = 'Waiting for Input')
Label_Speed.grid(row = 0, column = 1,  padx = 0)

Button_Top_Speed = Button(tab_Drive, text = 'Top Speed', command = Top_Speed)
Button_Top_Speed.grid(row = 2, column = 0, padx=5, pady=5)

Button_Slow_Speed = Button(tab_Drive, text = 'Slow Speed', command = Slow_Speed)
Button_Slow_Speed.grid(row = 2, column = 1, padx=5, pady=5)

Button_Angle_Speed = Button(tab_Drive, text = 'Angle Speed', command = Forward_Angle_Speed)
Button_Angle_Speed.grid(row = 2, column = 2, padx=5, pady=5)

forward_vel = Scale(tab_Drive, from_ = 100, to = -100, resolution = 0.1)
forward_vel.grid(row = 1, column = 2)

ang_vel = Scale(tab_Drive, from_ = 5, to = -5, resolution = 0.01)
ang_vel.grid(row = 1, column = 3)




def Enable_Motor():
    Label_Enable_Disbale = Label(tab_Drive,text = 'Motor Enabled')
    Label_Enable_Disbale.grid(row = 1,column = 1, padx = 0)

def Disable_Motor():
    Label_Enable_Disbale = Label(tab_Drive,text = 'Motor Disbaled')
    Label_Enable_Disbale.grid(row = 1,column = 1, padx = 0)


Label_Enable_Disbale = Label(tab_Drive,text = 'Motor Disbaled')
Label_Enable_Disbale.grid(row = 1, column = 1, padx = 0)


Button_Enable_Motor = Button(tab_Drive, text = 'Enable Motor', command = Enable_Motor)
Button_Enable_Motor.grid(row = 3, column = 0, padx=5, pady=5)

Button_Clean = Button(tab_Drive, text = 'QUIT', command = goodbye)
Button_Clean.grid(row = 3, column = 1, padx=5, pady=5)

Button_Disable_Motor = Button(tab_Drive, text = 'Disable Motor', command = Disable_Motor)
Button_Disable_Motor.grid(row = 3, column = 2, padx=5, pady=5)






# === WIDGETS FOR TAB THREE

def Enable_Motor():
    Label_Enable_Disbale = Label(tab_Drive,text = 'Motor Enabled')
    Label_Enable_Disbale.grid(row = 1,column = 1, padx = 0)

def Disable_Motor():
    Label_Enable_Disbale = Label(tab_Drive,text = 'Motor Disbaled')
    Label_Enable_Disbale.grid(row = 1,column = 1, padx = 0)

def Top_Speed():
    Label_Speed = Label(tab_Drive,text = '     Top Speed     ')
    Label_Speed.grid(row = 0,column = 1, padx = 0)

def Slow_Speed():
    Label_Speed = Label(tab_Drive,text = '      Slow Speed     ')
    Label_Speed.grid(row = 0,column = 1, padx = 0)

def Forward_Angle_Speed():
    Label_Speed = Label(tab_Drive,text = '    Angle Speed    ')
    Label_Speed.grid(row = 0,column = 1, padx = 0)

def goodbye():
    print("clean")
    

Label_Speed = Label(tab_Drive,text = 'Waiting for Input')
Label_Speed.grid(row = 0, column = 1,  padx = 0)

Label_Enable_Disbale = Label(tab_Drive,text = 'Motor Disbaled')
Label_Enable_Disbale.grid(row = 1, column = 1, padx = 0)

Button_Top_Speed = Button(tab_Drive, text = 'Top Speed', command = Top_Speed)
Button_Top_Speed.grid(row = 2, column = 0, padx=5, pady=5)

Button_Slow_Speed = Button(tab_Drive, text = 'Slow Speed', command = Slow_Speed)
Button_Slow_Speed.grid(row = 2, column = 1, padx=5, pady=5)

Button_Angle_Speed = Button(tab_Drive, text = 'Angle Speed', command = Forward_Angle_Speed)
Button_Angle_Speed.grid(row = 2, column = 2, padx=5, pady=5)

Button_Enable_Motor = Button(tab_Drive, text = 'Enable Motor', command = Enable_Motor)
Button_Enable_Motor.grid(row = 3, column = 0, padx=5, pady=5)

Button_Clean = Button(tab_Drive, text = 'QUIT', command = goodbye)
Button_Clean.grid(row = 3, column = 1, padx=5, pady=5)

Button_Disable_Motor = Button(tab_Drive, text = 'Disable Motor', command = Disable_Motor)
Button_Disable_Motor.grid(row = 3, column = 2, padx=5, pady=5)

forward_vel = Scale(tab_Drive, from_ = 100, to = -100, resolution = 0.1)
forward_vel.grid(row = 1, column = 2)

ang_vel = Scale(tab_Drive, from_ = 5, to = -5, resolution = 0.01)
ang_vel.grid(row = 1, column = 3)





# === WIDGETS FOR TAB FOUR
def Enable_MotorKD():
    Label_Enable_Disbale_KD = Label(tab_Kicker_Dribbler,text = 'Motors Enabled')
    Label_Enable_Disbale_KD.grid(row = 2,column = 1, padx = 0)

def Disable_MotorKD():
    Label_Enable_Disbale_KD = Label(tab_Kicker_Dribbler,text = 'Motors Disbaled')
    Label_Enable_Disbale_KD.grid(row = 2,column = 1, padx = 0)

Label_Enable_Disbale_KD = Label(tab_Kicker_Dribbler,text = 'Motor Disbaled')
Label_Enable_Disbale_KD.grid(row = 2, column = 1, padx = 0)

Button_Enable_Motors = Button(tab_Kicker_Dribbler, text = 'Enable Motors', command = Enable_MotorKD)
Button_Enable_Motors.grid(row = 4, column = 0, padx=5, pady=5)

Button_Disable_Motors = Button(tab_Kicker_Dribbler, text = 'Disable Motors', command = Disable_MotorKD)
Button_Disable_Motors.grid(row = 4, column = 2, padx=5, pady=5)




def Dribbler_ON():
    Label_Dribbler = Label(tab_Kicker_Dribbler,text = '     Dribbler On     ')
    Label_Dribbler.grid(row = 0,column = 1, padx = 0)

def Dribbler_OFF():
    Label_Dribbler = Label(tab_Kicker_Dribbler,text = '     Dribbler Off     ')
    Label_Dribbler.grid(row = 0,column = 1, padx = 0)

Label_Dribbler = Label(tab_Kicker_Dribbler,text = 'Waiting for Input')
Label_Dribbler.grid(row = 0, column = 1,  padx = 0)

Button_Dribbler_ON = Button(tab_Kicker_Dribbler, text = 'Dribbler On', command = Dribbler_ON)
Button_Dribbler_ON.grid(row = 3, column = 0, padx=5, pady=5)

Button_Dribbler_OFF = Button(tab_Kicker_Dribbler, text = 'Dribbler Off', command = Dribbler_OFF)
Button_Dribbler_OFF.grid(row = 3, column = 1, padx=5, pady=5)




def Kicker():
    Label_Kick = Label(tab_Kicker_Dribbler,text = '      Kicking     ')
    Label_Kick.grid(row = 1,column = 1, padx = 0)

    Label_Kick = Label(tab_Kicker_Dribbler,text = '      Kicking     ')
    Label_Kick.grid(row = 1,column = 1, padx = 0)

Label_Kick = Label(tab_Kicker_Dribbler,text = 'Waiting for Input')
Label_Kick.grid(row = 1, column = 1,  padx = 0)

Label_Kick = Label(tab_Kicker_Dribbler,text = 'Waiting for Input')
Label_Kick.grid(row = 1, column = 1,  padx = 0)

Button_Kick = Button(tab_Kicker_Dribbler, text = 'Kick', command = Kicker)
Button_Kick.grid(row = 3, column = 2, padx=5, pady=5)



Trig = True
def Check():
    if Trig == True: #GPIO.input(1):
        Label_Laser_Trigged = Label(tab_Kicker_Dribbler, text = 'High')
        Label_Laser_Trigged.grid(row = 1, column = 3, padx = 5, pady = 5)
    else:
        Label_Laser_Trigged = Label(tab_Kicker_Dribbler, text = 'Low')
        Label_Laser_Trigged.grid(row = 1, column = 3, padx = 5, pady = 5)
    #tab_Kicker_Dribbler.after(10, Check)

Label_Laser = Label(tab_Kicker_Dribbler, text = 'Trigged State')
Label_Laser.grid(row = 0, column = 3, padx = 5, pady = 5)

tab_Kicker_Dribbler.after(10, Check)



Button_Clean = Button(tab_Kicker_Dribbler, text = 'QUIT', command = goodbye)
Button_Clean.grid(row = 5, column = 1, padx=5, pady=5)




# === WIDGETS FOR TAB FIVE

Ball_Label = "No Ball"
Goal_Label = "No Goal"
def Running():
    if ball.get() == True:
        Ball_Label = "Ball"
    else:
        Ball_Label = "No Ball"

    if goal.get() == 1:
        Goal_Label = "Yellow Goal"
    elif goal.get() == 2:
        Goal_Label = "Blue Goal"
    else:
        Goal_Label = "No Goal"

    Label_Selection = Label(tab_Vision, text = "Running: " + Ball_Label + ", " + Goal_Label + ", " + str(Slider_Obstacules.get()))
    Label_Selection.grid(row=0, column=0, padx=5, pady=5)

ball = BooleanVar()

Radio_Button_No_Ball = Radiobutton(tab_Vision, text=" No Ball", variable = ball, value = False, command = Running)
Radio_Button_No_Ball.grid(row=1, column=0, padx=5, pady=5)
Radio_Button_Ball = Radiobutton(tab_Vision, text="Ball", variable = ball, value = True, command = Running)
Radio_Button_Ball.grid(row=2, column=0, padx=5, pady=5)


goal = IntVar()

Radio_Button_No_Goal = Radiobutton(tab_Vision, text=" No Goal", variable = goal, value = 0, command = Running)
Radio_Button_No_Goal.grid(row=1, column=1, padx=5, pady=5)
Radio_Button_Yellow_Goal = Radiobutton(tab_Vision, text="Yellow Goal", variable = goal, value = 1, command = Running)
Radio_Button_Yellow_Goal.grid(row=2, column=1, padx=5, pady=5)
Radio_Button_Blue_Goal = Radiobutton(tab_Vision, text="Blue Goal", variable = goal, value = 2, command = Running)
Radio_Button_Blue_Goal.grid(row=3, column=1, padx=5, pady=5)



Label_Obstacules = Label(tab_Vision, text="Number of Obstacules")
Label_Obstacules.grid(row=1, column=2, padx=5, pady=5)

Slider_Obstacules = Scale(tab_Vision, from_ = 3, to = 0, resolution = 1)
Slider_Obstacules.grid(row = 2, column = 2)

Button_Obstacule1 = Button(tab_Vision, text="Obstacules", command = Running)
Button_Obstacule1.grid(row=3, column=2, padx=5, pady=5)





def Start():
    Label_Running = Label(tab_Vision, text="Started")
    Label_Running.grid(row=0, column=2, padx=5, pady=5)

def Stop():
    Label_Running = Label(tab_Vision, text="Stopped")
    Label_Running.grid(row=0, column=2, padx=5, pady=5)


Label_Running = Label(tab_Vision, text="Not Running")
Label_Running.grid(row=0, column=2, padx=5, pady=5)

Button_Start = Button(tab_Vision, text="Start", command = Start)
Button_Start.grid(row=1, column=3, padx=5, pady=5)
Button_Stop = Button(tab_Vision, text="Stop", command = Stop)
Button_Stop.grid(row=2, column=3, padx=5, pady=5)

Button_Clean = Button(tab_Vision, text = 'QUIT', command = goodbye)
Button_Clean.grid(row = 4, column = 1, padx=5, pady=5)

Label_Selection = Label(tab_Vision, text = "Running: " + Ball_Label + ", " + Goal_Label + ", " + str(Slider_Obstacules.get()))
Label_Selection.grid(row=0, column=0, padx=5, pady=5)


#Alows window to appear in the middle of the screen
window_width = form.winfo_reqwidth()
window_height = form.winfo_reqheight()
position_right = int(form.winfo_screenwidth()/2.8 - window_width/2.8)
position_down = int(form.winfo_screenheight()/3 - window_height/3)
form.geometry("+{}+{}".format(position_right, position_down))

tab_GUI.pack(expand=1, fill='both')

form.mainloop()
