from setuptools import find_packages, setup

package_name = 'store_llm_agent'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['tests']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Store Cobot Team',
    maintainer_email='team@store.cobot',
    description='Voice STT, Mode FSM, RDFLib/SPARQL KG, and Orchestrator Agent',
    license='MIT',
    entry_points={
        'console_scripts': [
            'voice_node = store_llm_agent.nodes.voice_node:main',
            'orchestrator = store_llm_agent.nodes.orchestrator:main',
        ],
    },
)
