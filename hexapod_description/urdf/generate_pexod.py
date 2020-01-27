import os
from random import random
params ={
	        "generation_nb": 5,
            "min_len": 0.04,
            "max_len": 0.24,
	    }

def main(params):


     src = open("modules/robot.xacro","r")
     if src.mode == 'r':
         contents =src.read()
         print(contents)
     src.close()

     os.system('mkdir results')
     for i in range(0, params["generation_nb"]):

         scaled_value = params["min_len"] + (random() * (params["max_len"] - params["min_len"]))
         print(scaled_value)
         contents_modif = contents.replace('<xacro:property name="end_leg_lenght_0" value="0.140"/>', '<xacro:property name="end_leg_lenght_0" value="'+str(scaled_value)+'"/>')
         scaled_value = params["min_len"] + (random() * (params["max_len"] - params["min_len"]))
         contents_modif = contents_modif.replace('<xacro:property name="end_leg_lenght_1" value="0.140"/>', '<xacro:property name="end_leg_lenght_1" value="'+str(scaled_value)+'"/>')
         scaled_value = params["min_len"] + (random() * (params["max_len"] - params["min_len"]))
         contents_modif = contents_modif.replace('<xacro:property name="end_leg_lenght_2" value="0.140"/>', '<xacro:property name="end_leg_lenght_2" value="'+str(scaled_value)+'"/>')
         scaled_value = params["min_len"] + (random() * (params["max_len"] - params["min_len"]))
         contents_modif = contents_modif.replace('<xacro:property name="end_leg_lenght_3" value="0.140"/>', '<xacro:property name="end_leg_lenght_3" value="'+str(scaled_value)+'"/>')
         scaled_value = params["min_len"] + (random() * (params["max_len"] - params["min_len"]))
         contents_modif = contents_modif.replace('<xacro:property name="end_leg_lenght_4" value="0.140"/>', '<xacro:property name="end_leg_lenght_4" value="'+str(scaled_value)+'"/>')
         scaled_value = params["min_len"] + (random() * (params["max_len"] - params["min_len"]))
         contents_modif = contents_modif.replace('<xacro:property name="end_leg_lenght_5" value="0.140"/>', '<xacro:property name="end_leg_lenght_5" value="'+str(scaled_value)+'"/>')

         dest = open("modules/pexod.xacro","w+")
         dest.write(contents_modif)
         dest.close()

         cmd = 'rosrun xacro xacro --inorder -o '
         filename = 'results/pexod_' + str(i) +'.urdf'
         cmd = cmd + filename +' pexod.xacro'
         print(cmd)
         os.system(cmd)


if __name__== "__main__":
  main(params)
