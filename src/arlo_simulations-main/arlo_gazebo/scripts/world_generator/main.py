import random

def worldmaker(number):
    with open('Table1_c.world') as file:
        file_contents = file.read()

        line1 = """<model name='chips_can'>
        <pose>0.468093 0.095481 1.015 3.6e-05 3.4e-05 0.000128</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>0.468093 0.095481 1.015 3.6e-05 3.4e-05 0.000128</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 -9.8 0 -0 0</acceleration>
          <wrench>0 0 -2.009 0 -0 0</wrench>
        </link>
      </model>"""
        line1ran1=random.uniform(-0.3, 1.1)
        line1ran2=random.uniform(-0.48, 0.15)
        line1_replaced = """<model name='chips_can'>
        <pose>%f %f 1.015 3.6e-05 3.4e-05 0.000128</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>%f %f 1.015 3.6e-05 3.4e-05 0.000128</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 -9.8 0 -0 0</acceleration>
          <wrench>0 0 -2.009 0 -0 0</wrench>
        </link>
      </model>""" % (line1ran1,line1ran2,line1ran1,line1ran2)

        file_contents = file_contents.replace(line1, line1_replaced)

        line2 = """<model name='cracker_box'>
        <pose>0.433763 -0.406289 1.015 4e-05 1.2e-05 -1.56647</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>0.433763 -0.406289 1.015 4e-05 1.2e-05 -1.56647</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>-0.925778 -0.378365 -7.97364 -0.603097 0.67396 -0.735532</acceleration>
          <wrench>-0.380495 -0.155508 -3.27717 0 -0 0</wrench>
        </link>
      </model>"""
        line2ran1=random.uniform(-0.3, 1.1)
        line2ran2=random.uniform(-0.48, 0.15)
        line2_replaced = """<model name='cracker_box'>
        <pose>%f %f 1.015 4e-05 1.2e-05 -1.56647</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>%f %f 1.015 4e-05 1.2e-05 -1.56647</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>-0.925778 -0.378365 -7.97364 -0.603097 0.67396 -0.735532</acceleration>
          <wrench>-0.380495 -0.155508 -3.27717 0 -0 0</wrench>
        </link>
      </model>""" % (line2ran1,line2ran2,line2ran1,line2ran2)

        file_contents = file_contents.replace(line2, line2_replaced)

        line3 = """<model name='gelatin_box'>
        <pose>0.620469 -0.101894 1.05149 1.33887 1.57072 2.94057</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>0.620469 -0.101894 1.05149 1.33887 1.57072 2.94057</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 -9.8 -0 -0 -0.00112</acceleration>
          <wrench>0 0 -0.9506 0 -0 0</wrench>
        </link>
      </model>"""
        line3ran1=random.uniform(-0.3, 1.1)
        line3ran2=random.uniform(-0.48, 0.15)
        line3_replaced = """<model name='gelatin_box'>
        <pose>%f %f 1.05149 1.33887 1.57072 2.94057</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>%f %f 1.05149 1.33887 1.57072 2.94057</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 -9.8 -0 -0 -0.00112</acceleration>
          <wrench>0 0 -0.9506 0 -0 0</wrench>
        </link>
      </model>"""% (line3ran1,line3ran2,line3ran1,line3ran2)

        file_contents = file_contents.replace(line3, line3_replaced)

        line4 = """<model name='master_chef_can'>
        <pose>0.731323 -0.036647 1.015 0 -5e-06 0.002387</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>0.731323 -0.036647 1.015 0 -5e-06 0.002387</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>-0.331102 0.022642 -0.0116 -0.325977 1.51943 -0.000823</acceleration>
          <wrench>-0.137076 0.009374 -0.004802 0 -0 0</wrench>
        </link>
      </model>"""
        line4ran1=random.uniform(-0.3, 1.1)
        line4ran2=random.uniform(-0.48, 0.15)
        line4_replaced = """<model name='master_chef_can'>
        <pose>%f %f 1.015 0 -5e-06 0.002387</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>%f %f 1.015 0 -5e-06 0.002387</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>-0.331102 0.022642 -0.0116 -0.325977 1.51943 -0.000823</acceleration>
          <wrench>-0.137076 0.009374 -0.004802 0 -0 0</wrench>
        </link>
      </model>""" % (line4ran1,line4ran2,line4ran1,line4ran2)

        file_contents = file_contents.replace(line4, line4_replaced)


        line5 = """<model name='mustard_bottle'>
        <pose>0.25775 -0.080896 1.01837 -0.018547 0.024192 -0.022552</pose>
        <scale>1 1 1</scale>
        <link name='mustard_bottle::link'>
          <pose>0.25775 -0.080896 1.01837 -0.018547 0.024192 -0.022552</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 -9.8 0 -0 1e-06</acceleration>
          <wrench>0 0 -5.9094 0 -0 0</wrench>
        </link>
      </model>"""
        line5ran1=random.uniform(-0.3, 1.1)
        line5ran2=random.uniform(-0.48, 0.15)
        line5_replaced = """<model name='mustard_bottle'>
        <pose>%f %f 1.01837 -0.018547 0.024192 -0.022552</pose>
        <scale>1 1 1</scale>
        <link name='mustard_bottle::link'>
          <pose>%f %f 1.01837 -0.018547 0.024192 -0.022552</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 -9.8 0 -0 1e-06</acceleration>
          <wrench>0 0 -5.9094 0 -0 0</wrench>
        </link>
      </model>"""% (line5ran1,line5ran2,line5ran1,line5ran2)

        file_contents = file_contents.replace(line5, line5_replaced)
        

        '''line6 = """<model name='pitcher_base'>
        <pose>-59.8907 -37.5939 0.046706 -0.634096 0.082625 -0.405271</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>-59.8907 -37.5939 0.046706 -0.634096 0.082625 -0.405271</pose>
          <velocity>-0.006981 -0.013352 -0.014095 0.296758 -0.102473 -0.032866</velocity>
          <acceleration>-0.000855 -0.005421 14.5869 0.396906 -0.093153 -7e-06</acceleration>
          <wrench>-0.000209 -0.001323 3.55919 0 -0 0</wrench>
        </link>
      </model>"""
        line6ran1=random.uniform(-0.3, 1.1)
        line6ran2=random.uniform(-0.48, 0.15)
        line6_replaced = """<model name='pitcher_base'>
        <pose>%f %f 0.046706 -0.634096 0.082625 -0.405271</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>%f %f 0.046706 -0.634096 0.082625 -0.405271</pose>
          <velocity>-0.006981 -0.013352 -0.014095 0.296758 -0.102473 -0.032866</velocity>
          <acceleration>-0.000855 -0.005421 14.5869 0.396906 -0.093153 -7e-06</acceleration>
          <wrench>-0.000209 -0.001323 3.55919 0 -0 0</wrench>
        </link>
      </model>""" % (line6ran1,line6ran2,line6ran1,line6ran2)

        file_contents = file_contents.replace(line6, line6_replaced)  '''

        line7 = """<model name='potted_meat_can'>
        <pose>0.416297 -0.285898 1.015 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>0.416297 -0.285898 1.015 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>-0 0 0 -0 -0 -0</acceleration>
          <wrench>-0 0 0 0 -0 0</wrench>
        </link>
      </model>"""
        line7ran1=random.uniform(-0.3, 1.1)
        line7ran2=random.uniform(-0.48, 0.15)
        line7_replaced = """<model name='potted_meat_can'>
        <pose>%f %f 1.015 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>%f %f 1.015 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>-0 0 0 -0 -0 -0</acceleration>
          <wrench>-0 0 0 0 -0 0</wrench>
        </link>
      </model>"""  % (line7ran1,line7ran2,line7ran1,line7ran2)

        file_contents = file_contents.replace(line7, line7_replaced)

        line8 = """<model name='sugar_box'>
        <pose>0.765121 -0.275668 1.015 -2.2e-05 1.7e-05 0.004527</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>0.765121 -0.275668 1.015 -2.2e-05 1.7e-05 0.004527</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0.839974 1.9328 -4.48813 0.898867 -0.47608 -1.71262</acceleration>
          <wrench>0.431747 0.993459 -2.3069 0 -0 0</wrench>
        </link>
      </model>"""
        line8ran1=random.uniform(-0.3, 1.1)
        line8ran2=random.uniform(-0.48, 0.15)
        line8_replaced = """<model name='sugar_box'>
        <pose>%f %f 1.015 -2.2e-05 1.7e-05 0.004527</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>%f %f 1.015 -2.2e-05 1.7e-05 0.004527</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0.839974 1.9328 -4.48813 0.898867 -0.47608 -1.71262</acceleration>
          <wrench>0.431747 0.993459 -2.3069 0 -0 0</wrench>
        </link>
      </model>""" % (line8ran1,line8ran2,line8ran1,line8ran2)

        file_contents = file_contents.replace(line8, line8_replaced)

        line9 = """<model name='tomato_soup_can'>
        <pose>0.453708 -0.191474 1.015 -8e-05 -3.6e-05 0.073255</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>0.453708 -0.191474 1.015 -8e-05 -3.6e-05 0.073255</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 -9.8 0 -0 0</acceleration>
          <wrench>0 0 -3.4202 0 -0 0</wrench>
        </link>
      </model>"""
        line9ran1=random.uniform(-0.3, 1.1)
        line9ran2=random.uniform(-0.48, 0.15)
        line9_replaced = """<model name='tomato_soup_can'>
        <pose>%f %f 1.015 -8e-05 -3.6e-05 0.073255</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>%f %f 1.015 -8e-05 -3.6e-05 0.073255</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 -9.8 0 -0 0</acceleration>
          <wrench>0 0 -3.4202 0 -0 0</wrench>
        </link>
      </model>"""  % (line9ran1,line9ran2,line9ran1,line9ran2)

        file_contents = file_contents.replace(line9, line9_replaced)

        line10 = """<model name='tuna_fish_can'>
        <pose>0.610562 -0.215329 1.015 -0.000256 -0.000422 0.028013</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>0.610562 -0.215329 1.015 -0.000256 -0.000422 0.028013</pose>
          <velocity>-7e-06 -1e-05 -0.012122 -0.245815 -0.428152 0.000404</velocity>
          <acceleration>0 0 -9.8 0.000231 -0.000133 0</acceleration>
          <wrench>0 0 -1.6758 0 -0 0</wrench>
        </link>
      </model>"""
        line10ran1=random.uniform(-0.3, 1.1)
        line10ran2=random.uniform(-0.48, 0.15)
        
        line10_replaced = """<model name='tuna_fish_can'>
        <pose>%f %f 1.015 -0.000256 -0.000422 0.028013</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>%f %f 1.015 -0.000256 -0.000422 0.028013</pose>
          <velocity>-7e-06 -1e-05 -0.012122 -0.245815 -0.428152 0.000404</velocity>
          <acceleration>0 0 -9.8 0.000231 -0.000133 0</acceleration>
          <wrench>0 0 -1.6758 0 -0 0</wrench>
        </link>
      </model>""" % (line10ran1,line10ran2,line10ran1,line10ran2)

        file_contents = file_contents.replace(line10, line10_replaced)

        #print(file_contents)
        file_name="Table"+str(number)+"_c.world"
        f = open(file_name, "w+")
        f.write(file_contents)
        f.close
def main():
    for i in range(2,101):
        worldmaker(i)
    
if __name__ == "__main__":
    main()
        
        
