   M S B A M L   `   `   `  ����"  BodyExtractionAndHightlighting/    )BodyExtractionAndHightlighting.MainWindow   5   	   \ XPresentationFramework, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35R NWindowsBase, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35W SPresentationCore, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35D 9http://schemas.microsoft.com/winfx/2006/xaml/presentation    5   	   R NSystem.Xaml, Version=4.0.0.0, Culture=neutral, PublicKeyToken=b77a5c561934e089:x,http://schemas.microsoft.com/winfx/2006/xaml     5   	     � Title$  
MainWindow��5   	   $	��850��6   $	��700��6(   .��5      �  � RowDefinitions 5   
   �� 5      $
��Auto �6   �� 5      $
��Auto �6   �� 5	      $
��Auto �6   �� 5
      $
��Auto �6   �� 5      $
��Auto �6    � ColumnDefinitions 5   
   �� 5      $
��Auto �6   $	��150��6,   �� 5      $
��Auto �6   $	��300��6,   �� 5      $
��Auto �6   .+�5   
   �� $��0��6   $��0��6   $��Resolution:��6-   �� 5   
   $��0��6   $��1��6   $��Transparency of user:��6-   �� 5   
   $��0��6   $��2��6   $��Background:��6-   	� 5   
   -   5        $��fullHD��5      $��1��6$   $��0��61   $��1080p��6A    	� 	GroupName$ 
Resolution��6Q   6h   	� 5   
   -   5        $��	nonFullHD��5      $��2��6'   $��0��64   $��512x424��6D   $ 
Resolution��6V    d� 	IsChecked$
 TrueY�6m   6~   �� 5   
   -   5        $��userTransparency��5   �   $��2��6   $��1��6    � Value$	 150Q�6.    � Maximum$	 255Q�6:    �� TickFrequency$ 20Q�6H    �� TickPlacement$ BottomRight=�6[   	 �� IsSnapToTickEnabled	 . 6�   	� 5   
   -   5        $��BlackBG��5      $��1��6%   $��2��62   $��Black��6B   $ 
Background��6R   6i   	� 5   
   -   5        $��WhiteBG��5      $��2��6%   $��2��62   $��White��6B   $ 
Background��6R   6i   	� 5   
   -   5        $��CustomBG��5      $��3��6&   $��2��63   $��Custom��6C   $ 
Background��6T   $
 TrueY�6k   6|   �� 5!   
   $��4��6   $��3��6   
 K� ZIndex$
 1��62   .+�5%      �� -   5         ��Name$ 
imageCombi��5%       �� Left$ 20��6(   
 �� Top$ 10��69   $	��800��6I   $	��600��6U   5'      ��5)   
   �� 5*       �� ImageSource$# Resources\sharedWorkspace.jpg��6    j� Stretch$ Uniform=�6E   5,      5.      