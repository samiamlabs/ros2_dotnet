import sys
import os
import argparse
import logging
import shutil

from ament_index_python import get_package_prefix
from ament_index_python.packages import PackageNotFoundError

class UnityROS2LibCopier:
    def __init__(self, unity_project_path):
        self.ament_dependencies = [
            # 'rcl',
            # 'rcl_interfaces',
            # 'rmw',
            # 'rmw_implementation',
            # 'rmw_fastrtps_shared_cpp',
            # 'rosidl_generator_c',
            # 'rcl_logging_noop',
            # 'rmw_fastrtps_cpp',
            # 'rcutils',
            'rcldotnet',
            # 'fastrtps',
            'std_msgs',
            'geometry_msgs',
            'sensor_msgs',
            # 'tf2_msgs',
            'nav_msgs',
            'test_msgs'
        ]

        self.c_lib_source_dict = {}
        self.c_lib_destination_dir = unity_project_path + '/Assets/Plugins/x86_64'
        # self.c_lib_destination_dir = unity_project_path + '/Assets/Plugins'

        self.cs_lib_source_dict = {}
        self.cs_lib_destination_dir = unity_project_path + '/Assets/Plugins'

        self.__find_libs()

        
    def copy_files(self):
        self.__copy_files_from_dict(self.c_lib_source_dict, 
                                    self.c_lib_destination_dir)

        self.__copy_files_from_dict(self.cs_lib_source_dict,
                                    self.cs_lib_destination_dir)

    def __find_libs(self):
        self.__find_c_libs()
        self.__find_csharp_libs()

    def __find_c_libs(self):
        for package_name in self.ament_dependencies:
            try:
                package_install_path = get_package_prefix(package_name)
                package_lib_path = package_install_path + '/lib'
                for c_lib_file in os.listdir(package_lib_path):
                    if c_lib_file.endswith('.so'):
                        self.c_lib_source_dict[c_lib_file] = (
                            package_lib_path + '/' + c_lib_file)
            except PackageNotFoundError:
                print('{} not found!'.format(package_name))

    def __find_csharp_libs(self):
        for package_name in self.ament_dependencies:
            try:
                package_install_path = get_package_prefix(package_name)
                package_lib_path = package_install_path + '/lib/dotnet'
                if os.path.isdir(package_lib_path):
                    for csharp_lib_file in os.listdir(package_lib_path):
                        if csharp_lib_file.endswith('.dll'):
                            self.cs_lib_source_dict[csharp_lib_file] = (
                                package_lib_path + '/' + csharp_lib_file)
            except PackageNotFoundError:
                print('{} not found!'.format(package_name))

    def __copy_files_from_dict(self, source_dictionary, destination_directory):
        if not os.path.isdir(destination_directory):
            try:
               os.makedirs(destination_directory) 
            except OSError:
                logging.error('Failed to create directory {}'.format(destination_directory))
            else:
                logging.info('Successfully created the directory {}'.format(destination_directory))
            
        for filename, source_path in source_dictionary.items():
            destination_path = destination_directory + '/' + filename
            if self.__should_copy_file(source_path, destination_path):
                logging.info('Copying "{}"'.format(filename))
                shutil.copy2(source_path, destination_path)

    def __should_copy_file(self, source_path, destination_path):

        def does_not_exist(destination_path):
            return not os.path.isfile(destination_path)

        def newer_verson_available(source_path, destination_path):
            return (os.stat(source_path).st_mtime
                    - os.stat(destination_path).st_mtime > 0)

        return (does_not_exist(destination_path)
                or newer_verson_available(source_path, destination_path))

def parse_args(args):
    parser = argparse.ArgumentParser()

    parser.add_argument(
        '--unity-project', default=os.getcwd(), help='Path to Unity project'
    )

    return parser.parse_args(args)

def directory_is_a_unity_project(dir_path):
    unity_project_settings = dir_path + '/ProjectSettings/ProjectSettings.asset'
    return os.path.isfile(unity_project_settings)

def main():
    logging.basicConfig(level=logging.DEBUG)

    parsed = parse_args(sys.argv[1:])
    # parsed = parse_args(['--unity-project', 
    #                      '/home/sam/dyno/dotnet/unity_ws/src/unity/unity_ros2'])

    if directory_is_a_unity_project(parsed.unity_project):
        global lib_copier
        lib_copier = UnityROS2LibCopier(parsed.unity_project)
        lib_copier.copy_files()
    else:
        logging.error('The selected folder "{}" does not appear to be a Unity3D project.'.format(
            parsed.unity_project
        ))

if __name__ == '__main__':
    main()