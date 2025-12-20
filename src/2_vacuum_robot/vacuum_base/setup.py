import os
from setuptools import setup
from setuptools.command.install import install

package_name = "vacuum_base"


class PostInstallCommand(install):
    """在安装后创建可执行文件索引"""
    def run(self):
        install.run(self)
        # 从安装脚本路径推断安装根目录
        # 脚本安装在 bin/ 目录，我们需要找到包根目录
        if hasattr(self, 'install_scripts') and self.install_scripts:
            # install_scripts 通常是 <install_root>/bin
            # 我们需要找到 <install_root>/<package_name>
            scripts_dir = self.install_scripts
            # 向上两级到 install_root，然后到 package_name
            install_root = os.path.dirname(os.path.dirname(scripts_dir))
            package_install_dir = os.path.join(install_root, package_name)
            
            # 创建可执行文件索引目录
            executables_dir = os.path.join(
                package_install_dir,
                "share", "ament_index", "resource_index", "executables"
            )
            os.makedirs(executables_dir, exist_ok=True)
            # 创建索引文件
            index_file = os.path.join(executables_dir, package_name)
            with open(index_file, 'w') as f:
                f.write("serial_base_node\n")


setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="rest1",
    maintainer_email="rest1@example.com",
    description="Minimal serial base driver (cmd_vel <-> wheel speeds, odom+TF).",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "serial_base_node = vacuum_base.serial_base_node:main",
        ],
    },
    cmdclass={
        'install': PostInstallCommand,
    },
)
