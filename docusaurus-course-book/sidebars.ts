import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Part I: Foundations',
      link: {
        type: 'generated-index',
      },
      items: [
        'part-1-foundations/chapter-01-embodied-intelligence',
        'part-1-foundations/chapter-02-ecosystem',
      ],
    },
    {
      type: 'category',
      label: 'Part II: ROS 2 & Python',
      link: {
        type: 'generated-index',
      },
      items: [
        'part-2-ros2/chapter-03-architecture',
        'part-2-ros2/chapter-04-python',
        'part-2-ros2/chapter-05-urdf',
        'part-2-ros2/chapter-06-launch',
      ],
    },
    {
      type: 'category',
      label: 'Part III: Simulation',
      link: {
        type: 'generated-index',
      },
      items: [
        'part-3-simulation/chapter-07-gazebo',
        'part-3-simulation/chapter-08-sensor-simulation',
        'part-3-simulation/chapter-09-unity-visualization',
      ],
    },
    {
      type: 'category',
      label: 'Part IV: Isaac',
      link: {
        type: 'generated-index',
      },
      items: [
        'part-4-isaac/chapter-10-isaac-ecosystem',
        'part-4-isaac/chapter-11-isaac-sim',
        'part-4-isaac/chapter-12-isaac-ros-perception',
        'part-4-isaac/chapter-13-navigation-nav2',
      ],
    },
    {
      type: 'category',
      label: 'Part V: VLA',
      link: {
        type: 'generated-index',
      },
      items: [
        'part-5-vla/chapter-14-humanoid-kinematics',
        'part-5-vla/chapter-15-manipulation-grasping',
        'part-5-vla/chapter-16-voice-action-whisper',
        'part-5-vla/chapter-17-llms-robot-brains',
        'part-5-vla/chapter-18-multi-modal-interaction',
      ],
    },
    {
      type: 'category',
      label: 'Part VI: Deployment',
      link: {
        type: 'generated-index',
      },
      items: [
        'part-6-deployment/chapter-19-capstone-project',
        'part-6-deployment/chapter-20-edge-deployment',
        'part-6-deployment/chapter-21-physical-ai-lab',
        'part-6-deployment/chapter-22-future-physical-ai',
      ],
    },
  ],
};

export default sidebars;