   M S B A M L   `   `   `  ����   WpfApplication18    2Microsoft.Samples.Kinect.SkeletonBasics.MainWindow   \ XPresentationFramework, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35R NWindowsBase, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35W SPresentationCore, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35D 9http://schemas.microsoft.com/winfx/2006/xaml/presentation    R NSystem.Xaml, Version=4.0.0.0, Culture=neutral, PublicKeyToken=b77a5c561934e089:x,http://schemas.microsoft.com/winfx/2006/xaml     -     � Title$  Skeleton Basics��$	��735��$	��770�� � 	Resources    MediumGreyBrush �� Color  KinectPurpleBrush  KinectBlueBrush �� 
TargetType �� Property �� Value  SquareCheckBox �� 
TargetType �� Orientation �Name	 ��Name
 ��Name �� TextAlignment �� Triggers P� Property P� Value �� 
TargetName��%�  &	        &	      &	 *     '�� ?     &	 t     7  7  �� $ 	#ff6e6e6e���� $ 	#ff52318f���� $ 	#ff00BCF2���� " ��.���  �@J�SnapsToDevicePixels . �� " ��.���  � �� �� " ��.R�� .+��� $ 
Horizontal=�
������ .+�� $ SquareCheckBoxChecked��.+��� $	 CheckedNormal��$��Images\CheckedNormal.png��$
��None=�$��Center=��� $	 CheckedHover��$��Images\CheckedHover.png��$
��None=�$��Center=�$y�	Collapsed=�� $ SquareCheckBoxUnchecked��$y�	Collapsed=�.+��� $	 UncheckedNormal��$ ��Images\UncheckedNormal.png��$
��None=�$��Center=��� $	 UncheckedHover��$��Images\UncheckedHover.png��$
��None=�$��Center=�$y�	Collapsed=��� $
 SquareCheckBoxText��#��z��$
 Left=�$��Center=�8��  $��15(�$��9,0,0,0q� P�  �@d�	IsChecked$ falseY�. ��� $ SquareCheckBoxChecked�� � y�$ 	Collapsed=��� $ SquareCheckBoxUnchecked�� � y�$ Visible=�P�  �@J�IsMouseOver . . ��� $ CheckedNormal�� � y�$ 	Collapsed=��� $ UncheckedNormal�� � y�$ 	Collapsed=��� $ CheckedHover�� � y�$ Visible=��� $ UncheckedHover�� � y�$ Visible=��� 8  $ SquareCheckBoxText�� � ��.��� -   $��
layoutGrid��$��	10 0 10 0q� � RowDefinitions �� $
��Auto ��� $��* ��� $
��Auto ��� $
��Auto �.+�]� $��0��$��0 0 0 20q�.+��� $
��Left=�$��Images\Logo.png��$
��Fill=�$��32��$��81��$��0 10 0 5q��� $��Right=�$��0 0 -1 0q�$��Bottom=�#��[  $��Segoe UI)�$��18(�Skeleton Basics�� $��1��$��Images\Status.png��$
��None=�$��Center=�$��0 0 0 5q��� -   $��	Veri Onay��$��47��$	��Top=�$��75���� -   $��Veri Red��$��47��$	��Top=�$��75��(� $��1�� (� Stretch$ Uniform=�$��Center=�.���� -   $��Image��$	��640��$	��480���� -   $��checkArduinoConnected��$��2��#��[ $��Connect Arduino��$
��Auto��$��Right=�$��Center=�$��
0 10 10 10q��� -   $��checkBoxSeatedMode��$��2��#��[ $��Seated Mode��$
��Auto��$
��Left=�$��Center=�$��
0 10 10 10q��� -   $��	statusBar��$��3��$��Stretch=�$��Bottom=�
�������#��[  .@��� $��0 0 0 10q�.���� -	   $��statusBarText��$��-1 0 0 0q�20Click 'Seated' to change skeletal pipeline type!�� -
   $��RightShoulder��$
��Left=�$��23��$��
150,12,0,0q�$��2�� �� TextWrapping$
 Wrap=�$	��Top=�$	��100���� -   $��
RightElbow��$
��Left=�$��23��$��
250,12,0,0q�$��2��$
 Wrap=�$	��Top=�$	��100���� -   $��LeftShoulder��$
��Left=�$��23��$��
350,12,0,0q�$��2��$
 Wrap=�$	��Top=�$	��100���� -   $��	LeftElbow��$
��Left=�$��23��$��
450,12,0,0q�$��2��$
 Wrap=�$	��Top=�$	��100���� -   $��textBox1��$
��Left=�$��23��$��	567,0,0,0q�$��3��$
 Wrap=�$	��Top=�$	��100��