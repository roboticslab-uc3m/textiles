from setuptools import setup, find_packages

with open('requirements.txt') as f:
    requirements = f.read().splitlines()

try:
    import cv2
except ImportError:
    print("OpenCV 3 has not been found. It is advised to install it from sources" +
          "(check https://github.com/roboticslab-uc3m/textiles for more info).")
    print("I can install OpenCV with pip, but some functions might not be available.")
    ans = input("Proceed?[y/N]")
    if ans == 'y' or ans == 'Y':
        requirements.append('opencv')  # Fallback in case system-wide opencv is not installed

setup(name='textiles',
      version=0.1,
      url='https://github.com/roboticslab-uc3m/textiles',

      author='David Estevez',
      author_email="destevez@ing.uc3m.es",

      description='Algorithms for textile perception and manipulation using robots.',
      long_description=open('README.md').read(),

      packages=[package for package in find_packages() if 'app' not in package],
      package_data={'textiles.unfolding.apps'},

      entry_points={'console_scripts':
                    ['textiles_ironing_ExampleWrinkleDetector = textiles.ironing.apps.ExampleWrinkleDetector:main.start',
                     'textiles_ironing_RunClusteringPointCloudSegmentation = textiles.ironing.apps.RunClusteringPointCloudSegmentation:main.start',
                     'textiles_ironing_RunIroning = textiles.ironing.apps.RunIroning:main.start',
                     'textiles_ironing_RunLi = textiles.ironing.apps.RunLi:main.start',
                     'textiles_unfolding_generate_data = textiles.unfolding.apps.generate_data:main.start',
                     'textiles_unfolding_generate_figures = textiles.unfolding.apps.generate_figures:figure_7_2',
                     'textiles_unfolding_main = textiles.unfolding.apps.main:main',
                     'textiles_unfolding_evaluation = textiles.unfolding.apps.textiles_evaluation:main',
                     'textiles_unfolding_evaluation_stats = textiles.unfolding.apps.textiles_evaluation_stats:main',
                     'textiles_unfolding_evaluation_yarp = textiles.unfolding.apps.textiles_evaluation_yarp:main',
                     'textiles_unfolding_industrial = textiles.unfolding_industrial.apps.industrial_unfolding:main',
                     'textiles_unfolding_industrial_movement_test = textiles.unfolding_industrial.apps.movement_test:main',
                     'textiles_unfolding_industrial_evaluation = textiles.unfolding_industrial.apps.textiles_evaluation:main'
                     ],
                    'gui_scripts':
                    ['textiles_unfolding_evaluation_gui = textiles.unfolding.apps.textiles_evaluation_ui:main']}
      )