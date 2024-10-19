import pandas as pd
import numpy as np

# random state for reproducibility
np.random.seed(100)

# joint limits
c_arm_lateral_lmts = (-0.5, 0.5) # meters
c_arm_vertical_lmts = (0, 0.46)  # meters
c_arm_wigwag_lmts = (-10, 10)  # degrees
c_arm_horizontal_lmts = (0, 0.15)  # meters
c_arm_tilt_lmts = (-90, 270)  # degrees
c_arm_orbital_lmts = (-100, 100) # degrees

table_vertical_lmts = (0, 0.36)  # meters
table_trend_lmts = (-30, 30)  # degrees
table_tilt_lmts = (-20, 20)  # degrees
table_longitudinal_lmts = (0, 0.7)  # meters
table_transverse_lmts = (-0.13, 0.13)  # meters

# number of samples
num_of_samples = 1000

# random joint values
# random c-arm vertical joint values
c_arm_lateral = np.random.randint(low=int(c_arm_lateral_lmts[0]*100),
                                  high=int(c_arm_lateral_lmts[1]*100) + 1,
                                  size=num_of_samples,
                                  dtype=int)/100

# random c-arm vertical joint values
c_arm_vertical = np.random.randint(low=int(c_arm_vertical_lmts[0]*100),
                                    high=int(c_arm_vertical_lmts[1]*100) + 1,
                                    size=num_of_samples,
                                    dtype=int)/100

# random c-arm wigwag joint values
c_arm_wigwag = np.random.randint(low=int(c_arm_wigwag_lmts[0]),
                                    high=int(c_arm_wigwag_lmts[1]) + 1,
                                    size=num_of_samples,
                                    dtype=int)

# random c-arm horizontal joint values
c_arm_horizontal = np.random.randint(low=int(c_arm_horizontal_lmts[0]*100),
                                        high=int(c_arm_horizontal_lmts[1]*100) + 1,
                                        size=num_of_samples,
                                        dtype=int)/100

# random c-arm tilt joint values
c_arm_tilt = np.random.randint(low=int(c_arm_tilt_lmts[0]),
                               high=int(c_arm_tilt_lmts[1]) + 1,
                               size=num_of_samples,
                               dtype=int)

# random c-arm orbital joint values
c_arm_orbital = np.random.randint(low=int(c_arm_orbital_lmts[0]),
                                  high=int(c_arm_orbital_lmts[1]) + 1,
                                  size=num_of_samples,
                                  dtype=int)

# random table joint values
# random table vertical joint values
table_vertical = np.random.randint(low=int(table_vertical_lmts[0]*100),
                                    high=int(table_vertical_lmts[1]*100) + 1,
                                    size=num_of_samples,
                                    dtype=int)/100

# random table trend joint values
table_trend = np.random.randint(low=int(table_trend_lmts[0]),
                                    high=int(table_trend_lmts[1]) + 1,
                                    size=num_of_samples,
                                    dtype=int)

# random table tilt joint values
table_tilt = np.random.randint(low=int(table_tilt_lmts[0]),
                               high=int(table_tilt_lmts[1]) + 1,
                               size=num_of_samples,
                               dtype=int)

# random table longitudinal joint values
table_longitudinal = np.random.randint(low=int(table_longitudinal_lmts[0]*100),
                                       high=int(table_longitudinal_lmts[1]*100) + 1,
                                       size=num_of_samples,
                                       dtype=int)/100

# random table transverse joint values
table_transverse = np.random.randint(low=int(table_transverse_lmts[0]*100),
                                     high=int(table_transverse_lmts[1]*100) + 1,
                                     size=num_of_samples,
                                     dtype=int)/100

# create a dataframe
d = {'c_arm_lateral_m':c_arm_lateral,
     'c_arm_vertical_m':c_arm_vertical,
     'c_arm_wigwag_deg':c_arm_wigwag,
     'c_arm_horizontal_m':c_arm_horizontal,
     'c_arm_tilt_deg':c_arm_tilt,
     'c_arm_orbital_deg':c_arm_orbital,
     'table_vertical_m':table_vertical,
     'table_trend_deg':table_trend,
     'table_tilt_deg':table_tilt,
     'table_longitudinal_m':table_longitudinal,
     'table_transverse_m':table_transverse}

df = pd.DataFrame(data=d)

# save csv
df.to_csv("random_joint_values.csv")
