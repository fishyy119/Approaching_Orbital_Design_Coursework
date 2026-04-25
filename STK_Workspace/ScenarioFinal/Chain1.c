stk.v.12.0
WrittenBy    STK_v12.2.0

BEGIN Chain

    Name		 Chain1
    BEGIN Definition

        Object		 Constellation/Constellation1
        Object		 Satellite/Satellite2
        Type		 Chain
        FromOperator		 Or
        FromOrder		 1
        ToOperator		 Or
        ToOrder		 1
        Recompute		 Yes
        IntervalType		 0
        ComputeIntervalStart		 0
        ComputeIntervalStop		 86400
        ComputeIntervalPtr		
        BEGIN EVENTINTERVAL
            BEGIN Interval
                Start		 24 Apr 2026 04:00:00.000000000
                Stop		 25 Apr 2026 04:00:00.000000000
            END Interval
            IntervalState		 Explicit
        END EVENTINTERVAL

        ConstConstraintsByStrands		 Yes
        UseSaveIntervalFile		 No
        SaveIntervalFile		 C:\Matlab_workspace\Approaching_Orbital_Design_Coursework\STK_Workspace\ScenarioFinal\strand.int
        UseMinAngle		 No
        UseMaxAngle		 No
        UseMinLinkTime		 No
        LTDelayCriterion		 2
        TimeConvergence		 0.005
        AbsValueConvergence		 1e-14
        RelValueConvergence		 1e-08
        MaxTimeStep		 360
        MinTimeStep		 0.01
        UseLightTimeDelay		 Yes
        DetectEventsUsingSamplesOnly		 No
        UseLoadIntervalFile		 No
        BEGIN StrandObjIndexes
            StrandObj		 Place/Sanya
            StrandObj		 Place/Kashi
            StrandObj		 Place/Jiamusi
            StrandObj		 Satellite/Satellite2
        END StrandObjIndexes

        SaveMode		 1
        BEGIN StrandAccessesByIndex
            Strand		 0 3
            Start		  5.5285653635133094e+03
            Stop		  5.9643855529659904e+03
            Start		  2.5813624180602510e+04
            Stop		  2.6418948411796988e+04
            Start		  3.2371271418686239e+04
            Stop		  3.3225002848371012e+04
            Start		  3.9127712001169231e+04
            Stop		  3.9710665610913049e+04
            Start		  8.0468972572415194e+04
            Stop		  8.1171593774779903e+04
            Start		  8.7028552158711565e+04
            Stop		  8.7868158336581080e+04
            Start		  9.3900742623679922e+04
            Stop		  9.4397083954276750e+04
            Strand		 1 3
            Start		  1.1723336745143997e+04
            Stop		  1.2548320144746021e+04
            Start		  1.8350290346303656e+04
            Stop		  1.9204382070051703e+04
            Start		  2.5040414756439444e+04
            Stop		  2.5880360516899382e+04
            Start		  3.1705461821346405e+04
            Stop		  3.2563931262514503e+04
            Start		  3.8372400651045158e+04
            Stop		  3.9143612390620692e+04
            Start		  9.3610856318282342e+04
            Stop		  9.4268675374067054e+04
            Start		  1.0011635269686923e+05
            Stop		  1.0096835356919953e+05
            Start		  1.0677886927382147e+05
            Stop		  1.0726000000000000e+05
            Strand		 2 3
            Start		  7.0375614907750631e+01
            Stop		  3.9696354226163578e+02
            Start		  5.8644952927278109e+03
            Stop		  6.6240358960180956e+03
            Start		  1.2431259461348167e+04
            Stop		  1.3287444896331286e+04
            Start		  1.9074225547019407e+04
            Stop		  1.9897640167943096e+04
            Start		  2.5764831654545029e+04
            Stop		  2.6375186038094864e+04
            Start		  8.1197291684172902e+04
            Stop		  8.1685320424139005e+04
            Start		  8.7613115315402785e+04
            Stop		  8.8414346432438804e+04
            Start		  9.4205859582133518e+04
            Stop		  9.5060085262877154e+04
            Start		  1.0084421975468035e+05
            Stop		  1.0169666876981965e+05
        END StrandAccessesByIndex


    END Definition

    BEGIN Extensions

        BEGIN ExternData
        END ExternData

        BEGIN ADFFileData
        END ADFFileData

        BEGIN Desc
        END Desc

        BEGIN Crdn
        END Crdn

        BEGIN Graphics

            BEGIN Attributes

                StaticColor		 #00ffff
                AnimationColor		 #ff00ff
                AnimationLineWidth		 2
                StaticLineWidth		 3

            END Attributes

            BEGIN Graphics
                ShowGfx		 Off
                ShowStatic		 Off
                ShowAnimationHighlight		 On
                ShowAnimationLine		 On
                ShowLinkDirection		 Off
            END Graphics
        END Graphics

        BEGIN VO
        END VO

    END Extensions

END Chain

